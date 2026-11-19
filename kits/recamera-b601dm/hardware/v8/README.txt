reCamera B601 holder v8 - v7_tilt generator, tilt chosen for the real camera FOV
==============================================================================

Base: v7_tilt (v5 clip-on holder pitched down about the v7 pivot). v8 changes only the tilt
value and the analysis: DM gripper meshes, the real 1080p field of view, a simulated camera
view, and the armd camera_mount values. v5/ and v7_tilt/ are unchanged.

Recommended: TILT 38 deg. Alternative: TILT 42 deg.

Coordinates: end_link (DM MJCF end_link = armd URDF end_link = v7 "gripper_end"), mm.
Jaw tip at the origin, +X = approach direction, Y = jaw open/close axis, Z = gripper
thickness, holder on +Z.


1. Findings
-----------
1.1 Camera, sensor, FOV
  Sensor           OV5647, 2592x1944, 1.4 um pixels. Sources: Seeed wiki
                   https://wiki.seeedstudio.com/recamera_2002_series_hardware_and_specs/
                   (OV5647, 1/4", f 3.46 mm, F2.8, FOV 65 deg, distortion < 1 %), product page
                   https://www.seeed.cc/product/recamera-2002-series-open-source-ai-camera.
                   The spare reCamera (read-only) has libsns_ov5647.so and the OV5647 tuning
                   bin /mnt/cfg/param/ov_ov5647_sdr.bin. MIPI: 2 lanes RAW10. The sensor chip
                   ID was not read (that needs an I2C transaction).
  Native aspect    4:3 (2592x1944).
  Stream mode      1920x1080p30 (/proc/cvitek/vi: VI dev 1920x1080, VIFPS 30). Driver
                   reCamera-OS/external/SensorSupportList/sensor/cv182x/ov_ov5647/
                   ov5647_sensor_ctl.c:272-287: 0x3814/0x3815 = 0x11 (no binning or
                   skipping), window x 348..2275, y 434..1521. The 1080p image is therefore a
                   1:1 centre crop covering 74 % x 56 % of the array, which is a smaller FOV
                   than the full 4:3 sensor.
  Further crop     None. VI CHN crop N, VPSS group crop N, VPSS chn crop N
                   (/proc/cvitek/vi, /proc/cvitek/vpss). The ch2 stream is 1280x720, scaled
                   from 1920x1080. The ch0 detector input is 640x640 RGB with Aspect AUTO
                   (letterboxed 16:9 content). The 640x360 used by the kit is the same 16:9
                   content.
  FOV (derived)    The 1080p window is 2.688 x 1.512 mm. With f = 3.46 mm:
                     HFOV 42.47 deg (half 21.23), VFOV 24.65 deg (half 12.33), DFOV 48.1 deg.
                     fx = fy = 2471 px at 1920x1080, 1648 at 1280x720, 824 at 640x360.
                   Cross-check: on the full 4:3 array, f 3.46 mm gives DFOV 66.5 deg, close to
                   the 65 deg Seeed figure. The 65 deg figure therefore refers to the full
                   sensor, not the stream. NOT MEASURED: no calibration target was available.
  Intrinsics       rebot-companion tests/test_intrinsics.py K = [820 818 487 266] is a
                   synthetic fixture at 960x540 (W, H = 960, 540 in the test), not a reCamera
                   calibration. armd and rebot-vision use placeholders: fx = fy = 600 at
                   640x360, against the derived value of about 824.
  Lens position    ASSUMED at the centre of the 40 x 40 front face (as in v7); not checked
                   on a housing drawing. Section 3.2 shows the sensitivity to +-5 mm.
  Orientation      v5 on the real arm: image upright, gripper up/down = image short axis
                   (v7 README, not re-checked). VI channel Mirror = Y on the spare camera.

1.2 Why v7 predicted visible jaws (visibility.py, "v7 prediction vs real FOV")
  Main cause: v7 assumed a half vertical FOV of 17 deg (alternative 25). The real
  1080p-crop value is 12.33 deg.
                 tip angle below axis   v@17 (v7)   v@25 (v7)   v@12.33 (real)
    tilt 20      20.68 deg              1.235       0.809       1.728  out
    tilt 25      16.17 deg              0.948       0.622       1.327  out
    tilt 30      11.58 deg              0.670       0.439       0.938  edge
  (v = tan(angle) / tan(half VFOV), as a fraction of the half-height below the centre;
  1 = bottom edge.)
  The DM fingers taper to a point: z half-thickness is +-1.6 mm at x = -1, +-7.7 at x = -20
  and +-19.6 at x = -60. Only the last few millimetres of the jaw sit near the tip ray.
  At tilt 30 the simulated view shows 0.3 % of the image as gripper: a sliver at rows
  692-720 of 720 (camera_view_v7_tilt30_realFOV.png). Any of the following moves that sliver
  out of the frame: about 0.5 deg of extra error in mounted pitch, or the lens about 1-2 mm
  higher in the housing. With +5 mm lens height the model gives v = 1.17 (section 3.2). This
  is consistent with the report that the jaws are not visible. Which of these residual errors
  applies on the real arm is not determined.
  Secondary: the 40 deg half horizontal FOV v7 assumed is really 21.2 deg. This affects the
  lateral jaw view (section 2).

1.3 Gripper model (v7 weakness b), checked, not a cause
  - The DM gripper_base and finger meshes coincide with the RS gripper meshes v7 used, in
    the same frame. Identity transform, median nearest-neighbour distance 0.78 mm, p90
    1.47 mm. The RS MJCF itself says "DM 与 RS 使用同一夹爪" (DM and RS use the same gripper).
  - The DM MJCF end_link (link6 + 155.39 mm, quat 0 .707 0 .707) equals the armd URDF
    end_joint (xyz 0 0 0.15539, rpy 0 -1.5708 3.1415), and the jaw tip is at its origin.
  - The DM and RS link lengths differ (end 155.39 vs 166.21 mm from link6) only on the
    arm side of the gripper. The motor housing differs: DM link6 occupies x -160..-102,
    z +-38.4; RS motor_7 occupies x -157..-93, z +-41. The clamp interface (reference
    D435 mount, MJCF pose) was placed against the RS motor. On the DM motor the clamp may
    seat about 2.5 mm lower and up to about 3 mm different in x (NOT VERIFIED on the arm).
    That is about 1-2 deg of tip angle.
  - Side note: the finger and body meshes in rebot-arm-workspace/assets/urdf_b601dm/meshes/
    gripper (finger_L.stl X -160..-95, gripper_body.stl X -95..0) are mirrored in X relative
    to the DM MJCF and to link6. armd uses only the URDF kinematics, so this does not affect
    the camera values.

1.4 Image orientation and 16:9 (weaknesses c, e)
  The 16:9 stream is a crop of the 4:3 sensor. The crop is made by the sensor window, not
  by the ISP or VPSS (1.1). The short image axis spans the gripper up/down direction. This
  is the limiting axis, and v7 overestimated it by 38 % in tan.

1.5 armd camera_mount convention (src/kinematics.rs cam_mount)
  J5 axis in end_link at q6 = pi/2, computed from the DM chain: (0, -1, -0.0001). So the
  default mount (flip false, rot 0) has z_cam = +X, y_cam = -Y (image down = jaw axis
  -Y), x_cam = +Z. The holder gives z_cam = optical axis, y_cam = -Z-ish (image down toward
  the jaws) and x_cam = +Y. This requires rot_deg = +90. The kit's camera_mounts.json uses
  rot_deg 0 for v5 and v7. Those values describe a camera whose image-down direction is
  the jaw axis, which does not match this holder (NOT VERIFIED on the arm; the sign of
  rot depends on the image mirror setting, VI Mirror = Y).


2. Visibility model and candidates (visibility.py -> visibility_output.txt)
---------------------------------------------------------------------------
Method: lens pose from the cradle transform. Pinhole with the real FOV. Jaw tip = end_link
origin. Tip zone = finger points with x > -5 mm. Gripper coverage from a 128x72 ray grid
against the DM meshes; it agrees with the MuJoCo segmentation coverage. Per-finger opening
0 and 42.5 mm (85 mm span).

  candidate               tip v   cover(closed)  image top edge below      notes
                                                 gripper axis
  v5 (tilt 0)             +3.59   0 %            -12.3 deg (above)         jaws out
  v7 tilt 30              +0.94   0.3 %          17.7 deg                  edge
  tilt 34                 +0.63   4.9 %          21.7 deg
  tilt 36                 +0.48   7.7 %          23.7 deg                  worst case 0.87 (3.2)
  tilt 38 (recommended)   +0.33   10.8 %         25.7 deg                  worst case 0.72
  tilt 40                 +0.17   14.1 %         27.7 deg
  tilt 42 (alternative)   +0.02   17.7 %         29.7 deg                  tip at centre
  tilt 45                 -0.22   23.4 %         32.7 deg
  tilt 50                 -0.61   34.2 %         37.7 deg
  roll 90, tilt 30        +0.53   14.3 %         8.8 deg                   fingers leave image
                                                                           at 40 mm span (0.8 %)
  tilt 30, back 20 mm     +0.41   8.7 %          17.7 deg                  open u +-1.11; needs a
                                                                           new cradle over the clamp
  tilt 30, down 10 mm     +0.57   6.9 %          17.7 deg                  envelope clearance 6.1 mm
  tilt 30, fwd 15 mm      +1.48   0 %            17.7 deg                  worse
  tilt 35, back20 down5   -0.15   24.0 %         22.7 deg                  new cradle

Lateral limit, every candidate: when fully open (85 mm span) the fingertips are at
|u| = 1.1..1.4, outside the left and right edges, and the gripper covers 0 % of the image.
At about 98 mm from the lens the half-width is 38 mm. This follows from the 42.5 deg HFOV and
does not depend on the tilt. Tilt 38: both tip zones stay inside 80 % of the half-width up to
22 mm per finger (44 mm span). Rolling the camera 90 deg halves the lateral coverage (at
a 40 mm span the fingers are already outside). Moving the camera back 20 mm only gets to
|u| = 1.11 at full open.

Recommendation: tilt 38, landscape, camera position as v7.
  - Jaw tip at v = +0.33 (row 477 of 720 at 1280x720). Gripper covers 10.8 % (closed) /
    12.5 % (40 mm span) of the image. The highest gripper pixel is at v = +0.31, so the upper
    65 % of the image is free.
  - With lens height +-5 mm and pitch +-2 deg (9 combinations) the tip stays within
    v = -0.07 .. +0.72, inside 80 % of the half-height.
  - Cost: the image top edge is 25.7 deg below the gripper axis (v5: 12.3 deg above, v7 30:
    17.7 below), so the camera no longer sees straight ahead along the gripper. The body
    volume rises to 59.9 cm3 (v7 30: 55.7) and the bbox z max to 61.9 mm (v7 30: 56.9) in the
    mount frame; the rear rises further. Connector keep-outs and the insertion path are
    unchanged (all 0 mm3). Cable bend at the raised rear is not checked.
Alternative: tilt 42. Tip at the image centre (v = +0.02), v = -0.38..+0.42 over the same 9
  lens/pitch combinations (computed with visibility.evaluate; not in visibility_output.txt). Gripper covers 17.7 % (closed). Top edge 29.7 deg below
  the gripper axis.


3. Results
----------
3.1 validate_v8.py 38 42 (validation_output.txt)
                                                      tilt 38      tilt 42
  body / top bar: one valid solid each                yes          yes
  clamp interface y<=0 added/lost vs ref mm3          0 / 0        0 / 0
  clamp interface y<=0 added/lost vs v5 mm3           0 / 0        0 / 0
  body, top bar vs camera envelope mm3                0            0
  body, top bar vs B3 CAN cover mm3                   0            0
  top bar lift / 60 mm sweep / insertion / keep-outs  0            0
  retention region vs v5 (camera frame) mm3           0 / 0        0 / 0
  body volume mm3                                     59932.3      62082.9
  DM gripper clearance, openings 0/25/42.5 mm:
    camera envelope min mm                            16.209       16.222
    B3 CAN cover min mm                               37.187       39.295
    body above y = 0.5 min mm (points inside)         9.053 (0)    9.053 (0)
    top bar                                           >30          >30
    whole body: gripper points inside                 3390         3390
  FOV pyramid (+-12.33 V, +-21.23 H, 250 mm) intrusion body / top bar   0 / 0   0 / 0
  closest body/top point outside FOV, deg             48.3         46.4
  jaw tip v / u                                       +0.326/+0.012  +0.018/+0.012
  max per-finger opening, tips inside 80 % width      22.0 mm      21.5 mm
The 3390 points inside the whole body all lie in the clamp interface (y <= 0.5): the
body above y = 0.5 has 0. This is the reference D435 interface in its MJCF pose against
the DM motor housing (link6), the same situation as v7 against RS motor_7 (3383 points). It
is not introduced by v8. Line of sight lens -> tip at opening 0: "2 crossings at 96.1 of
98.5 mm". The closed fingertips themselves enclose the tip point within 2.4 mm of it; with
the jaws open the line of sight is clear.

3.2 Tip v sensitivity (lens height in the housing / mounted pitch error)
  tilt 30: -5/-2 +0.87  -5/0 +0.71  -5/+2 +0.55  0/-2 +1.11  0/0 +0.94  0/+2 +0.77
           +5/-2 +1.34  +5/0 +1.17  +5/+2 +1.00
  tilt 36: worst +0.87 (+5 mm, -2 deg)
  tilt 38: -5/-2 +0.25  -5/0 +0.09  -5/+2 -0.07  0/-2 +0.49  0/0 +0.33  0/+2 +0.17
           +5/-2 +0.72  +5/0 +0.56  +5/+2 +0.40
  tilt 40: worst +0.57

3.3 Simulated camera views (camera_view.py, MuJoCo 4.0.0 EGL, 1280x720, fovy 24.65 deg)
  camera_view_v8_tilt38.png        recommended; spans 0 / 40 / 85 mm
  camera_view_v8_tilt42.png        alternative
  camera_view_v7_tilt30_realFOV.png  v7 30 deg with the real FOV (sliver at the bottom edge)
  camera_view_roll90_tilt30.png    camera rolled 90 deg (portrait), v7 30 deg holder
  Pixel numbers in camera_view_output.txt. The red cross is the analytic tip projection; it
  sits on the apex of the rendered closed fingers in every image (checked visually).
  The holder never appears in the image (holder coverage 0.00 %).

3.4 Assembly previews: assembly_preview_v8_tilt38.png, assembly_preview_v8_tilt42.png
  (side view with the +-12.33 deg vertical FOV drawn in orange).


4. camera_mount for armd (recommended tilt 38; derived, NOT VERIFIED on the arm)
--------------------------------------------------------------------------------
  "camera_mount": {"flip": false, "rot_deg": 90.0, "tilt_deg": -38.0,
                   "offset_m": [-0.0731, 0.0005, 0.0660]}
  tilt 42: rot_deg 90.0, tilt_deg -42.0, offset_m [-0.0717, 0.0005, 0.0651]
  offset_m is the assumed lens centre in end_link. Derivation: validate_v8.py
  camera_mount() and README 1.5. Related grasp setup values:
    grip_pixel_norm ~ [0.506, 0.663] (jaw tip, tilt 38) / [0.506, 0.509] (tilt 42).
    fx = fy ~ 824 px at 640x360 (derived), against the placeholder 600.


5. Files
--------
  generate_recamera_mount.py      v7 generator; output names v8_tilt<N>, default 38
  optics.py                       FOV constants, DM gripper loader, lens frame, projection
  visibility.py / visibility_output.txt   candidate sweep and sensitivity
  validate_v8.py / validation_output.txt  B-rep, DM clearance, optics, FOV, camera_mount
  camera_view.py / camera_view_output.txt / camera_view_*.png   simulated camera view
  render_preview.py / assembly_preview_v8_tilt{38,42}.png
  generate_output.txt             generator run (CHECKS: ALL PASS for 38 and 42)
  reCamera_B601_Mount_v8_tilt{38,42}_{body,top_bar}.{step,stl}; md5sums.txt
Commands (from this directory; --no-project keeps uv out of any enclosing project):
  uv run --no-project --python 3.11 --with build123d --with trimesh --with matplotlib \
    --with scipy --with numpy --with manifold3d --with rtree python generate_recamera_mount.py 38
  (same environment) python validate_v8.py 38 42 ; python visibility.py ; python render_preview.py 38
  MUJOCO_GL=egl ... --with mujoco python camera_view.py
Print notes and assembly are as in v7 (two parts; same fasteners and nuts).


6. Open items (need the real arm / camera)
-----------------------------------------
  - Image from the arm with the v7 30 deg holder: the row of the jaw tip, if any, gives the
    combined lens and pitch error directly. Then confirm 38 deg on a print.
  - Intrinsics calibration (rebot-companion tools/calibrate_intrinsics.py) to replace the
    derived f and FOV.
  - Lens position in the housing (front-face centre assumed).
  - Clamp seating on the DM motor housing (designed against RS motor_7).
  - Sign of rot_deg (+90 vs -90) and of tilt_deg, from a camera-frame move on the arm; the
    effect of VI Mirror = Y.
  - Cable and connector access at the raised rear, holder stiffness at 38 deg, print
    orientation.
