# Example program for rebot-teleop: connect, print the arm state, go home, park.
# Copy this directory to /userdata/rebot-apps/hello/ and run:
#   /etc/init.d/K92rebot-teleop app-start hello
from rebot import Arm

with Arm() as arm:  # REBOT_HOST (127.0.0.1 under rebot-app-run); takes control, torque on
    state = arm.state()
    print("safety_state:", state.safety_state)
    print("positions:", [round(p, 3) for p in state.positions])
    arm.home()
    arm.park()
