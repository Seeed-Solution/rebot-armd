#!/bin/sh
# Joint-tracking A/B check before enabling position lead on a real B601-DM (docs/simulation.md,
# "Joint tracking A/B check"). Runs the joint-space A <-> B sequence of the 2026-11-22 incident
# through the supervisor output path on a first-order POS_VEL lag plant, tracking off vs on,
# and fails on any fault, an incomplete sequence, a moving-RMS reduction below the minimum
# (default 30 %), a higher peak or endpoint error, a peak issued velocity |v_out| above 1.5 x
# the peak with tracking off, a peak |lead| above max_lead, or an issued frame outside the
# output contract. Prints per joint, off and on: peak |v_out|, |a_out|, |lead|, |delta lead|
# per tick, and the withdrawal ticks of each run.
# No device access. Usage:
#   scripts/tracking_lead_sim.sh [profile.json] [plant tau J1..J7 in s, comma separated] \
#       [max_lead_rad for every lead joint] [minimum moving-RMS reduction %]
# An empty argument ("") keeps its default.
# profile.json: a B601-DM profile (for example the device's
#   /usr/share/rebot-teleop/profile/rebot-b601dm.json); its joint_tracking block is used as
#   written, with enabled forced off / on for the two runs. Default: the kit profile with
#   J1 = position_lead (lag 0.68 s, 0.03 rad), J2..J6 position.
# plant tau default: 0.68,0.65,0.71,0.17,0,0,0 (B601-DM measured lag seeds).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
if [ $# -ge 1 ] && [ -n "$1" ]; then
    REBOT_TRACKING_PROFILE=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
    export REBOT_TRACKING_PROFILE
fi
if [ $# -ge 2 ] && [ -n "$2" ]; then
    REBOT_TRACKING_PLANT_TAU=$2
    export REBOT_TRACKING_PLANT_TAU
fi
if [ $# -ge 3 ] && [ -n "$3" ]; then
    REBOT_TRACKING_MAX_LEAD=$3
    export REBOT_TRACKING_MAX_LEAD
fi
if [ $# -ge 4 ] && [ -n "$4" ]; then
    REBOT_TRACKING_MIN_RMS_REDUCTION_PCT=$4
    export REBOT_TRACKING_MIN_RMS_REDUCTION_PCT
fi
cd "$HERE/.."
exec timeout 1800 cargo test --locked --test test_joint_tracking tracking_lead_b601_joint_move_ab -- --nocapture --exact
