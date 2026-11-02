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
}

impl Default for EpisodeRecorder {
    fn default() -> Self {
        Self {
            state: EpisodeState::Idle,
            episode_id: String::new(),
            output_path: PathBuf::new(),
            frames: Vec::new(),
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
        self.frames.len()
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
        self.frames.clear();
        self.state = EpisodeState::Recording;
        Ok(())
    }

    /// Append a single telemetry/action frame during recording
    pub fn append_frame(&mut self, frame: RecordFrame) {
        if self.state == EpisodeState::Recording {
            self.frames.push(frame);
        }
    }

    /// Commit the recording and save to Parquet file
    pub fn save(&mut self) -> Result<PathBuf, RecordingError> {
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

        // Write to parquet
        match write_frames_to_parquet(&self.output_path, &self.frames) {
            Ok(()) => {
                self.state = EpisodeState::Committed;
                Ok(self.output_path.clone())
            }
            Err(e) => {
                // Return to recording or discarded on failure
                self.state = EpisodeState::Discarded;
                Err(RecordingError::Parquet(e))
            }
        }
    }

    /// Discard the current recording and drop buffered frames
    pub fn discard(&mut self) -> Result<(), RecordingError> {
        if self.state != EpisodeState::Recording {
            return Err(RecordingError::InvalidState {
                action: "discard".to_string(),
                state: self.state.clone(),
            });
        }

        self.frames.clear();
        self.state = EpisodeState::Discarded;
        Ok(())
    }
}
