//! State-file writer: ordered, newest state wins, no temp files left, invalid files moved aside.
use armd::persist::{self, Loaded, PendingWrite};

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_older_write_never_overwrites_a_newer_one() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("state.json");
    let old = PendingWrite::new(path.clone(), "\"old\"".into());
    let new = PendingWrite::new(path.clone(), "\"new\"".into());
    new.run().await.unwrap();
    old.run().await.unwrap();
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "\"new\"");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn concurrent_writes_end_with_the_last_prepared_state() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("state.json");
    // prepared in this order (as under the supervisor lock), queued from many tasks at once
    let writes: Vec<_> = (0..64).map(|i| PendingWrite::new(path.clone(), format!("{i}"))).collect();
    let tasks: Vec<_> = writes.into_iter().rev().map(|w| tokio::spawn(w.run())).collect();
    for t in tasks { t.await.unwrap().unwrap(); }
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "63");
    let names: Vec<_> = std::fs::read_dir(dir.path()).unwrap().map(|e| e.unwrap().file_name().to_string_lossy().into_owned()).collect();
    assert_eq!(names, ["state.json"], "no temp files left");
}

#[test]
fn invalid_file_is_moved_aside_before_any_write() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("state.json");
    std::fs::write(&path, "[1, 2").unwrap();
    let Loaded::Invalid(msg) = persist::load::<Vec<u32>>(&path, |_| Ok(())) else { panic!("invalid expected") };
    assert!(msg.contains(".bad-"), "{msg}");
    assert!(!path.exists());
    std::fs::write(&path, "[1]").unwrap();
    let Loaded::Invalid(_) = persist::load::<Vec<u32>>(&path, |v| if v.len() > 1 { Ok(()) } else { Err("short".into()) }) else { panic!() };
    let bad = std::fs::read_dir(dir.path()).unwrap().count();
    assert_eq!(bad, 2, "both invalid files kept under distinct names");
}
