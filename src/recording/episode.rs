use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use super::parquet_writer::{write_frames_to_parquet, ParquetRecordingError};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RecordFrame {
    pub timestamp: f64,
    pub observation_state: Vec<f32>,
    pub action: Vec<f32>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EpisodeState {
    Idle,
    Recording,
    Finalizing,
    Committed,
    Discarded,
}

#[derive(Debug, Error)]
pub enum RecordingError {
    #[error("Parquet error: {0}")]
    Parquet(#[from] ParquetRecordingError),

    #[error("Invalid state transition: cannot {action} while in state {state:?}")]
    InvalidState {
        action: String,
        state: EpisodeState,
    },

    #[error("No frames recorded: cannot save empty episode")]
    EmptyEpisode,

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

/// Episode recorder managing the recording lifecycle
#[derive(Debug, Clone)]
pub struct EpisodeRecorder {
    state: EpisodeState,
    episode_id: String,
    output_path: PathBuf,
    frames: Vec<RecordFrame>,
    saved_frame_count: usize,
}

impl Default for EpisodeRecorder {
    fn default() -> Self {
        Self {
            state: EpisodeState::Idle,
            episode_id: String::new(),
            output_path: PathBuf::new(),
            frames: Vec::new(),
            saved_frame_count: 0,
        }
    }
}

impl EpisodeRecorder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn state(&self) -> &EpisodeState {
        &self.state
    }

    pub fn frame_count(&self) -> usize {
        self.frames.len() + self.saved_frame_count
    }

    pub fn episode_id(&self) -> &str {
        &self.episode_id
    }

    pub fn output_path(&self) -> &PathBuf {
        &self.output_path
    }

    /// Start a new recording session
    pub fn start(
        &mut self,
        episode_id: Option<String>,
        output_path: Option<PathBuf>,
    ) -> Result<(), RecordingError> {
        let _ = self.start_taking_frames(episode_id, output_path)?;
        Ok(())
    }

    /// Start a new recording session, returning any previously buffered frames so the caller
    /// can drop them after releasing the supervisor write lock.
    pub fn start_taking_frames(
        &mut self,
        episode_id: Option<String>,
        output_path: Option<PathBuf>,
    ) -> Result<Vec<RecordFrame>, RecordingError> {
        if self.state == EpisodeState::Recording || self.state == EpisodeState::Finalizing {
            return Err(RecordingError::InvalidState {
                action: "start".to_string(),
                state: self.state.clone(),
            });
        }

        let id = episode_id.unwrap_or_else(|| {
            format!("episode_{}", chrono::Utc::now().format("%Y%m%d_%H%M%S"))
        });

        let path = output_path.unwrap_or_else(|| {
            PathBuf::from(format!("./data/episodes/{}.parquet", id))
        });

        self.episode_id = id;
        self.output_path = path;
        let prev = std::mem::take(&mut self.frames);
        self.saved_frame_count = 0;
        self.state = EpisodeState::Recording;
        Ok(prev)
    }

    /// Append a single telemetry/action frame during recording
    pub fn append_frame(&mut self, frame: RecordFrame) {
        if self.state == EpisodeState::Recording {
            self.frames.push(frame);
        }
    }

    /// Commit the recording and save to Parquet file
    pub fn save(&mut self) -> Result<PathBuf, RecordingError> {
        let (path, frames) = self.begin_save()?;
        let result = write_frames_to_parquet(&path, &frames);
        self.finish_save(frames, result)
    }

    /// First half of a save: move to `Finalizing` and hand out the buffered
    /// frames, so the caller can write them without holding the recorder (or any
    /// lock around it). Must be followed by [`finish_save`](Self::finish_save).
    pub fn begin_save(&mut self) -> Result<(PathBuf, Vec<RecordFrame>), RecordingError> {
        if self.state != EpisodeState::Recording {
            return Err(RecordingError::InvalidState {
                action: "save".to_string(),
                state: self.state.clone(),
            });
        }

        if self.frames.is_empty() {
            return Err(RecordingError::EmptyEpisode);
        }

        self.state = EpisodeState::Finalizing;
        self.saved_frame_count = 0;
        Ok((self.output_path.clone(), std::mem::take(&mut self.frames)))
    }

    /// [`begin_save`](Self::begin_save) when the frames are kept elsewhere (the control
    /// tick's recording ring): no empty check here, the caller checked the frame count.
    pub fn begin_finalize(&mut self) -> Result<(PathBuf, Vec<RecordFrame>), RecordingError> {
        if self.state != EpisodeState::Recording {
            return Err(RecordingError::InvalidState { action: "save".to_string(), state: self.state.clone() });
        }
        self.state = EpisodeState::Finalizing;
        self.saved_frame_count = 0;
        Ok((self.output_path.clone(), std::mem::take(&mut self.frames)))
    }

    /// Second half of a save: put the frames back and record the write outcome.
    pub fn finish_save(
        &mut self,
        frames: Vec<RecordFrame>,
        result: Result<(), ParquetRecordingError>,
    ) -> Result<PathBuf, RecordingError> {
        self.frames = frames;
        self.saved_frame_count = 0;
        match result {
            Ok(()) => {
                self.state = EpisodeState::Committed;
                Ok(self.output_path.clone())
            }
            Err(e) => {
                self.state = EpisodeState::Discarded;
                Err(RecordingError::Parquet(e))
            }
        }
    }

    /// Second half of a save when the caller already dropped the frames off-lock: records the
    /// saved frame count and write outcome without transferring or freeing `Vec<RecordFrame>`
    /// under the supervisor lock.
    pub fn finish_save_count(
        &mut self,
        frame_count: usize,
        result: Result<(), ParquetRecordingError>,
    ) -> Result<PathBuf, RecordingError> {
        self.saved_frame_count = frame_count;
        match result {
            Ok(()) => {
                self.state = EpisodeState::Committed;
                Ok(self.output_path.clone())
            }
            Err(e) => {
                self.state = EpisodeState::Discarded;
                Err(RecordingError::Parquet(e))
            }
        }
    }

    /// Discard the current recording and drop buffered frames
    pub fn discard(&mut self) -> Result<(), RecordingError> {
        let _ = self.discard_taking_frames()?;
        Ok(())
    }

    /// Discard the current recording and return any buffered frames so the caller can drop
    /// them after releasing the supervisor write lock.
    pub fn discard_taking_frames(&mut self) -> Result<Vec<RecordFrame>, RecordingError> {
        if self.state != EpisodeState::Recording {
            return Err(RecordingError::InvalidState {
                action: "discard".to_string(),
                state: self.state.clone(),
            });
        }

        let prev = std::mem::take(&mut self.frames);
        self.saved_frame_count = 0;
        self.state = EpisodeState::Discarded;
        Ok(prev)
    }
}
