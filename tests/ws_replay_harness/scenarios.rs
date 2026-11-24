//! 34 deterministic WS replay baseline scenarios covering every command family in A1b PR 1.

use serde_json::{json, Value};

use super::comparator::{ScriptStep, SessionSetup};
use super::seed::{SEEDED_EPISODE_ID, SEEDED_POLICY_ID, SEEDED_POSE_ID};

pub struct ScenarioSpec {
    pub id: &'static str,
    pub family: &'static str,
    pub description: &'static str,
    pub setup: SessionSetup,
    pub steps: Vec<ScriptStep>,
}

fn conn(client: &str) -> ScriptStep {
    ScriptStep::Connect { client: client.into(), query: String::new(), auth_header: None, expect_http_status: None }
}
fn conn_q(client: &str, query: &str, expect_http_status: Option<u16>) -> ScriptStep {
    ScriptStep::Connect { client: client.into(), query: query.into(), auth_header: None, expect_http_status }
}
fn send(client: &str, frame: Value) -> ScriptStep {
    ScriptStep::Send { client: client.into(), frame }
}
fn expect(client: &str, count: usize) -> ScriptStep {
    ScriptStep::ExpectControl { client: client.into(), count, capture_action: None }
}
fn expect_act(client: &str, alias: &str) -> ScriptStep {
    ScriptStep::ExpectControl { client: client.into(), count: 1, capture_action: Some(alias.into()) }
}
fn terminal(client: &str, alias: Option<&str>) -> ScriptStep {
    ScriptStep::ExpectTerminal { client: client.into(), action_alias: alias.map(str::to_string) }
}
fn no_reply(client: &str, wait_ms: u64) -> ScriptStep {
    ScriptStep::ExpectNoControl { client: client.into(), wait_ms }
}
fn close_client(client: &str) -> ScriptStep {
    ScriptStep::CloseClient { client: client.into() }
}
fn sleep(ms: u64) -> ScriptStep { ScriptStep::SleepMs { ms } }
fn rest(method: &str, path: &str, body: Option<Value>, capture_token: bool) -> ScriptStep {
    ScriptStep::Rest { method: method.into(), path: path.into(), body, auth_header: None, capture_token, record_body: true }
}
fn acquire(client: &str, label: &str, seq: u64) -> Vec<ScriptStep> {
    vec![send(client, json!({"type":"acquire_lease","client_id":label,"seq":seq})), expect(client, 1)]
}
fn cmd(client: &str, req: u64, path: &str, body: Value) -> ScriptStep {
    send(client, json!({"type":"command","request_id":req,"generation":format!("{{{{gen:{client}}}}}"),"epoch_id":"{{epoch}}","path":path,"body":body}))
}
fn free_cmd(client: &str, req: u64, path: &str, body: Value) -> ScriptStep {
    send(client, json!({"type":"command","request_id":req,"path":path,"body":body}))
}
fn torque_on(client: &str, req: u64) -> Vec<ScriptStep> {
    vec![cmd(client, req, "/api/v1/torque", json!({"enable":true})), expect(client, 1)]
}

fn dm_setup() -> SessionSetup { SessionSetup::default() }

pub fn all_scenarios() -> Vec<ScenarioSpec> {
    let inspect = json!([0.0, -1.167, -1.565, 1.435, 0.0, 0.0, 0.0]);
    let home = json!([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    let mut out = Vec::new();

    // 01: lease lifecycle
    out.push(ScenarioSpec {
        id: "01_lease_lifecycle", family: "lease",
        description: "acquire with seq, same-connection renew, release, double release, post-release LEASE_LOST",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl_a", 1),
            acquire("c0", "ctrl_a", 2),
            vec![
                send("c0", json!({"type":"release_lease","generation":"{{gen:c0}}"})), expect("c0", 1),
                send("c0", json!({"type":"release_lease","generation":"{{gen:c0}}"})), expect("c0", 1),
                cmd("c0", 1, "/api/v1/torque", json!({"enable":true})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 02: lease validation errors
    out.push(ScenarioSpec {
        id: "02_lease_validation", family: "lease",
        description: "invalid_client_id (empty & >128), non-boolean force, non-command without lease -> lease_lost gen 0",
        setup: dm_setup(),
        steps: vec![
            conn("c0"),
            send("c0", json!({"type":"acquire_lease","client_id":"","seq":1})), expect("c0", 1),
            send("c0", json!({"type":"acquire_lease","client_id":"a".repeat(129),"seq":2})), expect("c0", 1),
            send("c0", json!({"type":"acquire_lease","client_id":"ctrl","force":"yes","seq":3})), expect("c0", 1),
            send("c0", json!({"type":"preset","name":"home","generation":0,"epoch_id":"{{epoch}}"})), expect("c0", 1),
            close_client("c0"),
        ],
    });

    // 03: torque and /api/v1/arm alias
    out.push(ScenarioSpec {
        id: "03_torque_and_arm_alias", family: "torque",
        description: "torque & /api/v1/arm enable/disable, invalid enable type, non-object body",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/torque", json!({"enable":true})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/torque", json!({"enable":false})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/arm", json!({"enable":true})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/arm", json!({"enable":false})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/torque", json!({"enable":"nope"})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/torque", json!(42)), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 04: jog lifecycle & errors
    out.push(ScenarioSpec {
        id: "04_jog_lifecycle", family: "jog",
        description: "jog with duration_secs + terminal, jog with speed_scale, malformed positions, jog when disarmed",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/motion/jog", json!({"target_positions":inspect,"duration_secs":0.1})), expect("c0", 1),
            ],
            torque_on("c0", 2),
            vec![
                cmd("c0", 3, "/api/v1/motion/jog", json!({"target_positions":[0.05,0.0,0.0,0.0,0.0,0.0,0.0],"duration_secs":0.12})), expect_act("c0", "j1"), terminal("c0", Some("j1")),
                cmd("c0", 4, "/api/v1/motion/jog", json!({"target_positions":home,"speed_scale":0.8,"duration_secs":0.12})), expect_act("c0", "j2"), terminal("c0", Some("j2")),
                cmd("c0", 5, "/api/v1/motion/jog", json!({"target_positions":"bad"})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/motion/jog", json!({"target_positions":[null,0.0,0.0,0.0,0.0,0.0,0.0]})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 05: home & park/unload
    out.push(ScenarioSpec {
        id: "05_home_and_unload", family: "home_park",
        description: "jog away -> /api/v1/motion/home + terminal -> /api/v1/motion/unload",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/motion/jog", json!({"target_positions":[0.08,0.0,0.0,0.0,0.0,0.0,0.0],"duration_secs":0.1})), expect_act("c0", "j"), terminal("c0", Some("j")),
                cmd("c0", 3, "/api/v1/motion/home", json!({"duration_secs":0.12})), expect_act("c0", "h"), terminal("c0", Some("h")),
                cmd("c0", 4, "/api/v1/motion/unload", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 06: presets all variants
    out.push(ScenarioSpec {
        id: "06_presets_all_variants", family: "presets",
        description: "preset home, park, unload, custom jog with positions + speed_scale, missing positions reject",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"custom","positions":[0.05,0.0,0.0,0.0,0.0,0.0,0.0],"speed_scale":0.8})), expect_act("c0", "p1"), terminal("c0", Some("p1")),
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"home"})), expect_act("c0", "p2"), terminal("c0", Some("p2")),
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"park"})), expect_act("c0", "p3"),
                sleep(40),
                cmd("c0", 2, "/api/v1/motion/stop", json!({})), expect("c0", 1),
                terminal("c0", Some("p3")),
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"unload"})), expect("c0", 1),
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"bad_custom"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 07: move_to linear and joint
    out.push(ScenarioSpec {
        id: "07_move_to_linear_and_joint", family: "move_to",
        description: "move_to linear and path=joint with terminals, plus joint+speed and speed_scale validation rejects",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/motion/jog", json!({"target_positions":inspect,"duration_secs":0.15})), expect_act("c0", "go"), terminal("c0", Some("go")),
                cmd("c0", 3, "/api/v1/action/move_to", json!({"position":[0.20,0.0,0.28],"speed_scale":0.8,"tag":"lin"})), expect_act("c0", "m1"), terminal("c0", Some("m1")),
                cmd("c0", 4, "/api/v1/action/move_to", json!({"position":[0.18,0.05,0.28],"path":"joint","speed_scale":0.8,"tag":"jnt"})), expect_act("c0", "m2"), terminal("c0", Some("m2")),
                cmd("c0", 5, "/api/v1/action/move_to", json!({"position":[0.18,0.05,0.28],"path":"joint","speed":0.2})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/action/move_to", json!({"position":[0.18,0.05,0.28],"speed_scale":2.5})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 08: move_relative and gripper
    out.push(ScenarioSpec {
        id: "08_move_relative_and_gripper", family: "move_relative",
        description: "move_relative base frame + terminal, gripper open/close + baseline reject, gripper_zero_check",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/motion/jog", json!({"target_positions":inspect,"duration_secs":0.15})), expect_act("c0", "go"), terminal("c0", Some("go")),
                cmd("c0", 3, "/api/v1/action/move_relative", json!({"frame":"base","d":[0.0,0.0,0.02],"speed_scale":0.8})), expect_act("c0", "mr"), terminal("c0", Some("mr")),
                cmd("c0", 4, "/api/v1/action/gripper", json!({"command":"open","speed":1.5})), expect_act("c0", "g1"), terminal("c0", Some("g1")),
                cmd("c0", 5, "/api/v1/action/gripper", json!({"command":"move","fraction":0.5,"speed_scale":0.8})), expect_act("c0", "g2"), terminal("c0", Some("g2")),
                cmd("c0", 6, "/api/v1/action/gripper", json!({"command":"open","baseline":"x"})), expect("c0", 1),
                cmd("c0", 7, "/api/v1/action/gripper_zero_check", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 09: poses CRUD and move_to_pose
    out.push(ScenarioSpec {
        id: "09_poses_crud_and_move", family: "poses",
        description: "poses/save, poses/rename, move_to_pose + terminal, poses/delete, missing pose reject",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/poses/save", json!({"name":"p_alpha","positions":[0.05,0.0,0.0,0.0,0.0,0.0,0.0]})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/poses/rename", json!({"id":"{{pose_id}}","name":"p_beta"})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/action/move_to_pose", json!({"name":"p_beta","speed_scale":0.9})), expect_act("c0", "mp"), terminal("c0", Some("mp")),
                cmd("c0", 5, "/api/v1/poses/delete", json!({"name":"p_beta"})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/poses/delete", json!({"name":"p_missing"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 10: grasp dry_run (without and with lease) and place
    out.push(ScenarioSpec {
        id: "10_grasp_dry_run_and_place", family: "place_grasp",
        description: "grasp dry_run without lease (exempt), grasp dry_run with lease + mock detector, place validation",
        setup: SessionSetup { seed_poses: true, mock_detector: true, ..dm_setup() },
        steps: [
            vec![
                conn("c0"),
                send("c0", json!({"type":"command","request_id":1,"path":"/api/v1/action/grasp","body":{"dry_run":true,"position":[0.20,0.0,0.10]}})), expect("c0", 1),
            ],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 2),
            vec![
                cmd("c0", 3, "/api/v1/action/grasp", json!({"dry_run":true,"position":[0.20,0.0,0.10]})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/action/place", json!({})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/action/place", json!({"position":[2.0,2.0,2.0]})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 11: lease-free depth measure and teach commands
    out.push(ScenarioSpec {
        id: "11_depth_and_teach_lease_free", family: "teach_depth",
        description: "lease-free /api/v1/grasp/measure and teach dataset/record/episode commands without acquiring lease",
        setup: SessionSetup { mock_detector: true, ..dm_setup() },
        steps: vec![
            conn("c0"),
            free_cmd("c0", 1, "/api/v1/grasp/measure", json!({})), expect("c0", 1),
            free_cmd("c0", 2, "/api/v1/datasets/create", json!({"name":"demo_ds"})), expect("c0", 1),
            free_cmd("c0", 2, "/api/v1/datasets/create", json!({"name":"dup_req"})), expect("c0", 1),
            send("c0", json!({"type":"command","request_id":3,"client_id":"teach_cli","path":"/api/v1/record/start","body":{"dataset":"demo_ds","task":"pick"}})), expect("c0", 1),
            sleep(80),
            send("c0", json!({"type":"command","request_id":4,"client_id":"teach_cli","path":"/api/v1/record/stop","body":{"save":true}})), expect("c0", 1),
            free_cmd("c0", 5, "/api/v1/episodes/update", json!({"id":"{{episode_id}}","task":"pick_v2"})), expect("c0", 1),
            free_cmd("c0", 6, "/api/v1/episodes/delete", json!({"id":"{{episode_id}}"})), expect("c0", 1),
            free_cmd("c0", 7, "/api/v1/datasets/delete", json!({"name":"demo_ds"})), expect("c0", 1),
            close_client("c0"),
        ],
    });

    // 12: replay seeded episode
    out.push(ScenarioSpec {
        id: "12_replay_episode", family: "replay",
        description: "replay seeded parquet episode + terminal, plus missing episode rejection",
        setup: SessionSetup { seed_episode: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/action/replay", json!({"episode":SEEDED_EPISODE_ID,"speed":1.0})), expect_act("c0", "rp"), terminal("c0", Some("rp")),
                cmd("c0", 3, "/api/v1/action/replay", json!({"episode":"01ARZ3NDEKTSV4RRFFQ69G5FAV"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 13: sequences save, run, delete
    out.push(ScenarioSpec {
        id: "13_sequences_save_run_delete", family: "sequence",
        description: "sequences/save with seeded pose + wait step, run_sequence + terminal, sequences/delete",
        setup: SessionSetup { seed_poses: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/sequences/save", json!({"name":"seq_demo","steps":[{"type":"pose","pose_id":SEEDED_POSE_ID,"speed_scale":1.0},{"type":"wait","s":0.05}]})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/action/run_sequence", json!({"id":"{{sequence_id}}"})), expect_act("c0", "sq"), terminal("c0", Some("sq")),
                cmd("c0", 4, "/api/v1/sequences/delete", json!({"id":"{{sequence_id}}"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 14: servo explicit, cartesian_joint_stream, implicit positions, command /api/v1/motion/servo, preset precedence
    out.push(ScenarioSpec {
        id: "14_servo_explicit_and_aliases", family: "servo",
        description: "explicit servo, cartesian_joint_stream alias, implicit positions frame, /api/v1/motion/servo, preset>servo precedence",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":1,"positions":[0.01,0.0,0.0,0.0,0.0,0.0,0.0]})), no_reply("c0", 60),
                send("c0", json!({"mode":"cartesian_joint_stream","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":2,"positions":[0.02,0.0,0.0,0.0,0.0,0.0,0.0]})), no_reply("c0", 60),
                send("c0", json!({"generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":3,"positions":[0.03,0.0,0.0,0.0,0.0,0.0,0.0]})), no_reply("c0", 60),
                cmd("c0", 2, "/api/v1/motion/servo", json!({"targets":[0.02,0.0,0.0,0.0,0.0,0.0,0.0]})), expect("c0", 1),
                send("c0", json!({"type":"preset","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","name":"custom","positions":[0.0,0.0,0.0,0.0,0.0,0.0,0.0]})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 15: servo validation & alias precedence edge cases
    out.push(ScenarioSpec {
        id: "15_servo_validation_and_precedence", family: "servo",
        description: "missing_generation on servo & cartesian_joint_stream vs lease_lost on implicit positions, sequence_id errors, disarmed & stale epoch",
        setup: dm_setup(),
        steps: [
            vec![
                conn("c0"),
                send("c0", json!({"type":"servo","sequence_id":1,"positions":home})), expect("c0", 1),
                send("c0", json!({"mode":"cartesian_joint_stream","sequence_id":1,"positions":home})), expect("c0", 1),
                send("c0", json!({"sequence_id":1,"positions":home})), expect("c0", 1),
            ],
            acquire("c0", "ctrl", 1),
            vec![
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","positions":home})), expect("c0", 1),
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":999999,"sequence_id":1,"positions":home})), expect("c0", 1),
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":1,"positions":home})), expect("c0", 1),
            ],
            torque_on("c0", 1),
            vec![
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":1,"positions":home})), expect("c0", 1),
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":2,"positions":home})), no_reply("c0", 50),
                send("c0", json!({"type":"servo","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","sequence_id":2000,"positions":home})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 16: policy shadow start and stop
    out.push(ScenarioSpec {
        id: "16_policy_shadow_start_stop", family: "policy",
        description: "policy/start in shadow mode with seeded RMLP pack, policy/stop + terminal, POLICY_NOT_FOUND",
        setup: SessionSetup { seed_policy: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/action/policy/start", json!({"policy_id":"no_such_pack","mode":"shadow"})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/action/policy/start", json!({"policy_id":SEEDED_POLICY_ID,"mode":"shadow","max_seconds":5.0})), expect_act("c0", "pol"),
                sleep(60),
                cmd("c0", 3, "/api/v1/action/policy/stop", json!({})), expect("c0", 1), terminal("c0", Some("pol")),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 17: config keepout and collision
    out.push(ScenarioSpec {
        id: "17_config_keepout_and_collision", family: "config",
        description: "workspace/keepout persisted update + revision mismatch, safety/collision disable & mock enable",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/workspace/keepout", json!({"boxes":[{"name":"box1","min_m":[0.30,-0.30,0.0],"max_m":[0.40,-0.20,0.10]}],"expected_revision":1})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/workspace/keepout", json!({"boxes":[],"expected_revision":1})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/safety/collision", json!({"enabled":false,"sensitivity":"low"})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/safety/collision", json!({"enabled":true})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 18: config gripper_calib, grasp/setup, tool commissioning
    out.push(ScenarioSpec {
        id: "18_config_gripper_grasp_commissioning", family: "config",
        description: "gripper/calibration, grasp/setup, tool/commissioning/confirm persisted updates & rejects",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/gripper/calibration", json!({"jaw_opening":{"closed_m":0.0,"open_m":0.065},"offset_rad":0.0})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/grasp/setup", json!({"table_z_m":0.02})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/tool/commissioning/confirm", json!({"confirm":false})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/tool/commissioning/confirm", json!({"confirm":true,"tool_id":"wrong","tool_config_digest":"00"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 19: gravity_comp mode
    out.push(ScenarioSpec {
        id: "19_gravity_comp_mode", family: "gravity_comp",
        description: "mode/gravity_comp enable/disable, invalid enable/scale/damping_kd validation",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/mode/gravity_comp", json!({"enable":true,"scale":0.8,"damping_kd":0.2})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/mode/gravity_comp", json!({"enable":false})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/mode/gravity_comp", json!({"enable":"yes"})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/mode/gravity_comp", json!({"enable":true,"scale":"bad"})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/mode/gravity_comp", json!({"enable":true,"damping_kd":"bad"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 20: calibration apply and restore
    out.push(ScenarioSpec {
        id: "20_calibration_apply_and_restore", family: "calibration",
        description: "calibration/apply dry_run & invalid body, calibration/restore non-object & missing backup",
        setup: SessionSetup { seed_calibration: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/calibration/apply", json!({"dry_run":true,"intrinsics":{"fx":600.0,"fy":600.0,"cx":320.0,"cy":240.0,"width":640,"height":480},"camera_mount":{"offset_m":[0.0,0.0,0.05],"q_end_cam":[0.0,0.0,0.0,1.0]},"table_z_m":0.0})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/calibration/apply", json!({"bogus":1})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/calibration/restore", json!(123)), expect("c0", 1),
                cmd("c0", 4, "/api/v1/calibration/restore", json!({"backup":"missing.json"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 21: calibration motor_zero, joint_zero/restore, mit_timeout, mit_qualification
    out.push(ScenarioSpec {
        id: "21_calibration_motor_zero_and_mit", family: "calibration",
        description: "calibration/motor_zero, joint_zero/restore, mit_timeout, mit_qualification",
        setup: SessionSetup { seed_calibration: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/calibration/motor_zero", json!({"joints":[1],"dry_run":true})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/calibration/motor_zero", json!({"joints":[]})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/calibration/joint_zero/restore", json!({"backup":"none.json"})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/calibration/mit_timeout", json!({})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/calibration/mit_qualification", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 22: calibration & misc actions (calib_hover, gripper_calibrate, measure_multiview)
    out.push(ScenarioSpec {
        id: "22_calibration_and_misc_actions", family: "calibration",
        description: "calib_hover, gripper_calibrate, measure_multiview validation & mock responses",
        setup: SessionSetup { seed_calibration: true, ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/action/calib_hover", json!({"resume":"nonexistent"})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/action/gripper_calibrate", json!({})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/action/measure_multiview", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 23: E-stop both aliases and clear_fault
    out.push(ScenarioSpec {
        id: "23_estop_aliases_and_clear_fault", family: "estop",
        description: "estop & e_stop aliases, stale epoch EPOCH_MISMATCH, torque refused in EStop, clear_fault valid & invalid mit_recovery",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                send("c0", json!({"type":"estop"})), expect("c0", 1),
                send("c0", json!({"type":"command","request_id":1,"generation":"{{gen:c0}}","epoch_id":0,"path":"/api/v1/torque","body":{"enable":true}})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/torque", json!({"enable":true})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/safety/clear_fault", json!({"mit_recovery":"bad"})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/safety/clear_fault", json!({})), expect("c0", 1),
                send("c0", json!({"type":"e_stop","request_id":99})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/safety/clear_fault", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 24: E-stop injected via REST owner event
    out.push(ScenarioSpec {
        id: "24_estop_injected_rest_owner_event", family: "estop",
        description: "REST POST /api/v1/safety/e_stop owner event -> WS heartbeat epoch_sync + stale EPOCH_MISMATCH + clear_fault",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 1),
            vec![
                rest("POST", "/api/v1/safety/e_stop", None, false),
                send("c0", json!({"type":"heartbeat","generation":"{{gen:c0}}","epoch_id":1})), expect("c0", 1),
                send("c0", json!({"type":"command","request_id":2,"generation":"{{gen:c0}}","epoch_id":1,"path":"/api/v1/torque","body":{"enable":true}})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/safety/clear_fault", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 25: Stop cancels motion + Stop validation errors
    out.push(ScenarioSpec {
        id: "25_stop_cancels_motion_and_errors", family: "stop",
        description: "urgent /api/v1/motion/stop cancels slow jog + terminal cancelled; lease_lost, epoch_mismatch, dup req_id, bad body",
        setup: dm_setup(),
        steps: [
            vec![
                conn("c0"),
                send("c0", json!({"type":"command","request_id":1,"generation":0,"epoch_id":"{{epoch}}","path":"/api/v1/motion/stop","body":{}})), expect("c0", 1),
            ],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 2),
            vec![
                sleep(50),
                cmd("c0", 3, "/api/v1/motion/jog", json!({"target_positions":[0.4,0.0,0.0,0.0,0.0,0.0,0.0],"duration_secs":2.0})), expect_act("c0", "slow"),
                sleep(50),
                cmd("c0", 4, "/api/v1/motion/stop", json!({})), expect("c0", 1), terminal("c0", Some("slow")),
                cmd("c0", 4, "/api/v1/motion/stop", json!({})), expect("c0", 1),
                send("c0", json!({"type":"command","request_id":5,"generation":"{{gen:c0}}","epoch_id":999999,"path":"/api/v1/motion/stop","body":{}})), expect("c0", 1),
                cmd("c0", 6, "/api/v1/motion/stop", json!("not_an_object")), expect("c0", 1),
                // Read-only observation probe verifies post-Stop steady-state safety_state == "Armed"
                // and no lingering active action without sending a second mutating Stop command.
                sleep(40),
                ScriptStep::CheckState,
                close_client("c0"),
            ],
        ].concat(),
    });

    // 26: Stop vs ordinary request_id independence & torque disable after stop
    out.push(ScenarioSpec {
        id: "26_stop_ordinary_reqid_and_torque_disable", family: "stop",
        description: "urgent Stop request_id=100 does not block ordinary request_id=11; duplicate ordinary fails; torque disable succeeds",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            torque_on("c0", 10),
            vec![
                cmd("c0", 100, "/api/v1/motion/stop", json!({})), expect("c0", 1),
                cmd("c0", 11, "/api/v1/torque", json!({"enable":false})), expect("c0", 1),
                cmd("c0", 11, "/api/v1/torque", json!({"enable":false})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 27: Auth/PIN explicit mode, unlock, privilege checks
    out.push(ScenarioSpec {
        id: "27_auth_pin_explicit_and_privileges", family: "auth",
        description: "--operator-pin + ?auth=explicit: OPERATOR_REQUIRED before auth, bad token -> viewer, REST unlock + WS auth -> operator",
        setup: SessionSetup { operator_pin: Some("246810".into()), ..dm_setup() },
        steps: [
            vec![conn_q("c0", "auth=explicit", None)],
            vec![
                send("c0", json!({"type":"acquire_lease","client_id":"op","force":true,"seq":1})), expect("c0", 1),
                free_cmd("c0", 1, "/api/v1/datasets/create", json!({"name":"unauth_ds"})), expect("c0", 1),
                free_cmd("c0", 2, "/api/v1/record/stop", json!({"save":false})), expect("c0", 1),
            ],
            acquire("c0", "op", 2),
            vec![
                cmd("c0", 3, "/api/v1/workspace/keepout", json!({"boxes":[],"expected_revision":1})), expect("c0", 1),
                cmd("c0", 4, "/api/v1/mode/gravity_comp", json!({"enable":true})), expect("c0", 1),
                send("c0", json!({"type":"auth","seq":1,"token":"wrong-token"})), expect("c0", 1),
                rest("POST", "/api/v1/auth/unlock", Some(json!({"pin":"000000"})), false),
                rest("POST", "/api/v1/auth/unlock", Some(json!({"pin":"246810"})), true),
                send("c0", json!({"type":"auth","seq":2,"token":"{{token}}"})), expect("c0", 1),
                cmd("c0", 5, "/api/v1/workspace/keepout", json!({"boxes":[],"expected_revision":1})), expect("c0", 1),
                free_cmd("c0", 6, "/api/v1/datasets/create", json!({"name":"authed_ds"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 28: Auth clear_fault after E-stop requires operator
    out.push(ScenarioSpec {
        id: "28_auth_clear_fault_after_estop", family: "auth",
        description: "viewer cannot clear_fault latched EStop; succeeds after REST unlock + WS auth",
        setup: SessionSetup { operator_pin: Some("246810".into()), ..dm_setup() },
        steps: [
            vec![conn_q("c0", "auth=explicit", None)],
            acquire("c0", "op", 1),
            vec![
                send("c0", json!({"type":"estop"})), expect("c0", 1),
                cmd("c0", 1, "/api/v1/safety/clear_fault", json!({})), expect("c0", 1),
                rest("POST", "/api/v1/auth/unlock", Some(json!({"pin":"246810"})), true),
                send("c0", json!({"type":"auth","seq":1,"token":"{{token}}"})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/safety/clear_fault", json!({})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 29: Heartbeat renew, omitted gen, epoch_sync, and lease expiry
    out.push(ScenarioSpec {
        id: "29_heartbeat_renew_sync_and_expiry", family: "heartbeat",
        description: "heartbeat valid & omitted gen, stale epoch -> epoch_sync, short lease_timeout_ms expiry -> lease_lost expired",
        setup: SessionSetup { profile: "rebot_dm_short_lease".into(), ..dm_setup() },
        steps: [
            vec![conn("c0")],
            acquire("c0", "hb_client", 1),
            vec![
                send("c0", json!({"type":"heartbeat","generation":"{{gen:c0}}","epoch_id":"{{epoch}}"})), no_reply("c0", 60),
                send("c0", json!({"type":"heartbeat","epoch_id":"{{epoch}}"})), no_reply("c0", 60),
                send("c0", json!({"type":"heartbeat","generation":"{{gen:c0}}","epoch_id":999999})), expect("c0", 1),
                sleep(550),
                send("c0", json!({"type":"heartbeat","generation":"{{gen:c0}}","epoch_id":"{{epoch}}"})), expect("c0", 1),
                cmd("c0", 1, "/api/v1/torque", json!({"enable":true})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 30: Preemption, eviction notice, and cooldown
    out.push(ScenarioSpec {
        id: "30_preemption_eviction_and_cooldown", family: "preemption",
        description: "c0 holds lease, c1 non-force controller_busy, c1 force preempts (c0 gets lease_lost preempted), c0 re-preempt hits cooldown",
        setup: dm_setup(),
        steps: [
            vec![conn("c0"), conn("c1")],
            acquire("c0", "holder_a", 1),
            vec![
                send("c1", json!({"type":"acquire_lease","client_id":"holder_b","seq":1})), expect("c1", 1),
                send("c1", json!({"type":"acquire_lease","client_id":"holder_b","force":true,"seq":2})), expect("c1", 1), expect("c0", 1),
                send("c0", json!({"type":"acquire_lease","client_id":"holder_a","force":true,"seq":2})), expect("c0", 1),
                cmd("c0", 1, "/api/v1/torque", json!({"enable":true})), expect("c0", 1),
                close_client("c0"), close_client("c1"),
            ],
        ].concat(),
    });

    // 31: REST lease restrictions while WS holds lease
    out.push(ScenarioSpec {
        id: "31_rest_lease_restrictions", family: "preemption",
        description: "REST mutating endpoints rejected with connection-lease requirement while WS holds lease",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ws_holder", 1),
            vec![
                rest("POST", "/api/v1/torque", Some(json!({"enable":true})), false),
                rest("POST", "/api/v1/motion/jog", Some(json!({"target_positions":home})), false),
                rest("POST", "/api/v1/motion/home", Some(json!({})), false),
                rest("POST", "/api/v1/action/move_to", Some(json!({"position":[0.2,0.0,0.2]})), false),
                rest("POST", "/api/v1/mode/gravity_comp", Some(json!({"enable":true})), false),
                send("c0", json!({"type":"release_lease","generation":"{{gen:c0}}"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 32: Disconnect releases lease & stops active motion
    out.push(ScenarioSpec {
        id: "32_disconnect_releases_lease_and_stops", family: "disconnect",
        description: "c0 acquires lease + starts 2s jog + disconnects -> c1 immediately acquires lease non-force",
        setup: dm_setup(),
        steps: [
            vec![conn("c0"), conn("c1")],
            acquire("c0", "transient", 1),
            torque_on("c0", 1),
            vec![
                cmd("c0", 2, "/api/v1/motion/jog", json!({"target_positions":[0.4,0.0,0.0,0.0,0.0,0.0,0.0],"duration_secs":2.0})), expect_act("c0", "disc_jog"),
                close_client("c0"),
                terminal("c1", Some("disc_jog")),
                sleep(40),
            ],
            acquire("c1", "successor", 1),
            vec![
                // Read-only probe confirms steady-state Armed and no active action after disconnect.
                ScriptStep::CheckState,
                close_client("c1"),
            ],
        ].concat(),
    });

    // 33: Unknown frames, malformed JSON, unknown command/action paths, missing req_id, bad duration_secs
    out.push(ScenarioSpec {
        id: "33_unknown_frames_and_paths", family: "dispatch",
        description: "malformed text ignored; unknown type without gen -> lease_lost, with valid gen+epoch -> ignored; unknown command vs action errors; req_id & duration_secs validation",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                ScriptStep::SendRaw { client: "c0".into(), text: "not-valid-json{{{".into() },
                no_reply("c0", 40),
                send("c0", json!({"type":"unknown_msg_kind","foo":1})), expect("c0", 1),
                send("c0", json!({"type":"unknown_msg_kind","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","foo":1})),
                no_reply("c0", 40),
                send("c0", json!({"type":"command","generation":"{{gen:c0}}","epoch_id":"{{epoch}}","path":"/api/v1/torque","body":{"enable":true}})), expect("c0", 1),
                cmd("c0", 1, "/api/v1/does_not_exist", json!({})), expect("c0", 1),
                cmd("c0", 2, "/api/v1/action/does_not_exist", json!({})), expect("c0", 1),
                cmd("c0", 3, "/api/v1/torque", json!({"enable":true,"duration_secs":"nope"})), expect("c0", 1),
                close_client("c0"),
            ],
        ].concat(),
    });

    // 34: WS upgrade query parsing (?obs_hz=abc 400, ?obs_hz=5 telemetry pacing)
    out.push(ScenarioSpec {
        id: "34_ws_upgrade_query_and_obs_hz", family: "upgrade_telemetry",
        description: "invalid obs_hz returns HTTP 400; obs_hz=5 streams bounded monotonic 7-DOF telemetry with verified joint positions",
        setup: dm_setup(),
        steps: vec![
            conn_q("bad", "obs_hz=abc", Some(400)),
            conn_q("c0", "obs_hz=5", Some(101)),
            ScriptStep::CollectTelemetry { client: "c0".into(), duration_ms: 650, min_count: 2, max_count: 5 },
            close_client("c0"),
        ],
    });

    // 35: Explicit WS Close frame -> clean server EOF teardown + lease release
    out.push(ScenarioSpec {
        id: "35_server_close_and_teardown", family: "close",
        description: "explicit WS Close frame drives clean server EOF teardown (distinct from Frame/TransportError) and releases lease",
        setup: dm_setup(),
        steps: [
            vec![conn("c0"), conn("c1")],
            acquire("c0", "closing_holder", 1),
            vec![
                ScriptStep::SendCloseFrame { client: "c0".into(), code: 1000, reason: "client_done".into() },
                ScriptStep::ExpectClose { client: "c0".into() },
                sleep(40),
            ],
            acquire("c1", "after_close", 1),
            vec![close_client("c1")],
        ].concat(),
    });

    out
}

/// Companion session used when `policy-runtime` feature is disabled (`--no-default-features`),
/// pinning exact legacy `Unknown action` rejection on policy endpoints.
pub fn policy_disabled_scenario() -> ScenarioSpec {
    ScenarioSpec {
        id: "16_policy_disabled_reject",
        family: "policy",
        description: "without policy-runtime feature, policy/start and policy/stop return 400 INVALID_COMMAND Unknown action",
        setup: dm_setup(),
        steps: [
            vec![conn("c0")],
            acquire("c0", "ctrl", 1),
            vec![
                cmd("c0", 1, "/api/v1/action/policy/start", json!({"policy_id":"shadow_demo","mode":"shadow"})),
                expect("c0", 1),
                cmd("c0", 2, "/api/v1/action/policy/stop", json!({})),
                expect("c0", 1),
                cmd("c0", 3, "/api/v1/action/policy/start", json!("not_an_object")),
                expect("c0", 1),
                close_client("c0"),
            ],
        ]
        .concat(),
    }
}
