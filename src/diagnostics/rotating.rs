//! Size-rotating append-only file: `name`, `name.1` … `name.<keep>`.

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

/// Appends records to `path`; before a record would grow the file past `max_bytes`, the
/// file is renamed to `path.1` (`path.1` to `path.2`, …, the oldest dropped) and a new one
/// is started. Each record is one `write` on an unbuffered file, so it reaches the page
/// cache before the call returns; [`RotatingFile::sync_data`] also puts it on the medium.
#[derive(Debug)]
pub struct RotatingFile {
    path: PathBuf,
    max_bytes: u64,
    keep: usize,
    file: File,
    size: u64,
}

impl RotatingFile {
    /// Opens (appends to) `path`. `keep` old files are kept next to it.
    pub fn open(path: impl Into<PathBuf>, max_bytes: u64, keep: usize) -> io::Result<Self> {
        let path = path.into();
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        let size = file.metadata()?.len();
        Ok(Self { path, max_bytes, keep, file, size })
    }

    /// Bytes in the current file.
    pub fn size(&self) -> u64 {
        self.size
    }

    /// `path.<n>` (n ≥ 1) for an old file; `path` for n = 0.
    pub fn numbered(path: &Path, n: usize) -> PathBuf {
        if n == 0 {
            return path.to_path_buf();
        }
        let mut name = path.as_os_str().to_owned();
        name.push(format!(".{n}"));
        PathBuf::from(name)
    }

    fn rotate(&mut self) -> io::Result<()> {
        if self.keep == 0 {
            // Append mode writes at the (new) end, so truncating in place restarts the file.
            self.file.set_len(0)?;
        } else {
            for n in (1..self.keep).rev() {
                let from = Self::numbered(&self.path, n);
                if from.exists() {
                    std::fs::rename(&from, Self::numbered(&self.path, n + 1))?;
                }
            }
            std::fs::rename(&self.path, Self::numbered(&self.path, 1))?;
            self.file = OpenOptions::new().create(true).append(true).open(&self.path)?;
        }
        self.size = 0;
        Ok(())
    }

    /// Writes one record (rotating first when it would not fit). A record larger than
    /// `max_bytes` still goes into a file of its own.
    pub fn write_record(&mut self, record: &[u8]) -> io::Result<()> {
        if self.size > 0 && self.size + record.len() as u64 > self.max_bytes {
            self.rotate()?;
        }
        self.file.write_all(record)?;
        self.size += record.len() as u64;
        Ok(())
    }

    /// `fdatasync` the current file.
    pub fn sync_data(&self) -> io::Result<()> {
        self.file.sync_data()
    }
}

impl Write for RotatingFile {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.write_record(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Files `path.<keep>` … `path.1`, `path` that exist, oldest first.
pub fn files_oldest_first(path: &Path, keep: usize) -> Vec<PathBuf> {
    (0..=keep).rev().map(|n| RotatingFile::numbered(path, n)).filter(|p| p.exists()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read(p: &Path) -> String {
        std::fs::read_to_string(p).unwrap_or_default()
    }

    #[test]
    fn rotates_by_size_and_keeps_the_newest_files() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("armd.log");
        let mut f = RotatingFile::open(&path, 100, 2).unwrap();
        // 10-byte records: 10 per file.
        for i in 0..45 {
            f.write_record(format!("rec {i:05}\n").as_bytes()).unwrap();
            assert!(f.size() <= 100);
        }
        let cur = read(&path);
        let one = read(&RotatingFile::numbered(&path, 1));
        let two = read(&RotatingFile::numbered(&path, 2));
        assert_eq!(cur.lines().count(), 5);
        assert!(cur.starts_with("rec 00040"));
        assert_eq!(one.lines().count(), 10);
        assert!(one.starts_with("rec 00030"));
        assert_eq!(two.lines().count(), 10);
        assert!(two.starts_with("rec 00020"));
        assert!(!RotatingFile::numbered(&path, 3).exists(), "only keep=2 old files");
        assert_eq!(std::fs::metadata(&path).unwrap().len(), 50);
        assert_eq!(files_oldest_first(&path, 2).len(), 3);
    }

    #[test]
    fn reopening_appends_and_counts_the_existing_size() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bb.jsonl");
        RotatingFile::open(&path, 30, 1).unwrap().write_record(b"0123456789\n").unwrap();
        let mut f = RotatingFile::open(&path, 30, 1).unwrap();
        assert_eq!(f.size(), 11);
        f.write_record(b"0123456789\n").unwrap();
        f.write_record(b"0123456789\n").unwrap(); // 33 > 30: rotates first
        assert_eq!(read(&path), "0123456789\n");
        assert_eq!(read(&RotatingFile::numbered(&path, 1)).len(), 22);
        f.sync_data().unwrap();
    }

    #[test]
    fn oversized_record_and_keep_zero() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("x.log");
        let mut f = RotatingFile::open(&path, 8, 0).unwrap();
        f.write_record(b"0123456789abc\n").unwrap();
        assert_eq!(f.size(), 14, "a record bigger than max_bytes is written whole");
        f.write_record(b"next\n").unwrap();
        assert_eq!(read(&path), "next\n");
        assert!(!RotatingFile::numbered(&path, 1).exists());
    }

    #[test]
    fn write_trait_is_one_record_per_call() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("w.log");
        let mut f = RotatingFile::open(&path, 12, 1).unwrap();
        writeln!(f, "aaaa").unwrap(); // "aaaa" and "\n" may be two writes; both fit
        f.write_all(b"bbbbbbbbb\n").unwrap();
        assert_eq!(read(&RotatingFile::numbered(&path, 1)), "aaaa\n");
        assert_eq!(read(&path), "bbbbbbbbb\n");
    }
}
