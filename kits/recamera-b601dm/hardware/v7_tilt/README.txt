reCamera B601 holder v7_tilt - v5 clip-on holder with the camera pitched down
============================================================================

Base: v5 (enclosed holder, separate body and top_bar; camera retained by the
rear stop, front lower lip, front side guards and the removable top bar; the
camera's internal screw holes are not used). v6_tilt was based on v4 and is
obsolete.

Coordinates: gripper frame (MJCF body gripper_end), mm. Jaw tip at origin,
X forward toward the tip, Z up, Y lateral. STEP/STL files are in the same
mount frame as the v5 STEP files.

Image orientation (observed on the real arm with v5): image upright; gripper
up/down = image vertical (short) axis, left/right = image horizontal (long)
axis.


1. What changed vs v5
---------------------
- New parameter TILT_DEG (generate_recamera_mount.py, CLI argument).
  All cradle features (rear stop, rails, side guards and ears, bottom
  support, front lip, camera cavity, bottom port, ear bores, nut pockets),
  the top bar, the camera envelope and the B3 CAN cover are rotated by
  TILT_DEG about the cradle's lateral axis. The optical axis goes forward
  and down.
- Pivot: front-bottom edge of the v5 rails / bottom support (cradle-local
  y = 43, z = 0.7). The rear of the cradle rises; no tilted part goes below
  the v5 support level.
- Support wedge: 2D convex hull of the v5 support band and the tilted
  support band (side profile), extruded over the rear-stop width (46.6 mm).
  It is solid. It keeps the v5 footprint, so the v5 bridge and the clamp
  interface connect as in v5.
- Bottom port under the CAN rear cover is also cut through the wedge so it
  stays open.
- Unchanged from v5: clamp interface (mount y <= 0, copied from
  D435_Gemini2_Mount.step), bridge, all retention geometry relative to the
  camera, the top bar (same volume, same holes), clearance 0.3 mm,
  3.2 mm clearance holes, nut pockets, CAN cover roll orientation.

Variants and files:
  TILT  body                                      top bar                                      preview
  20    reCamera_B601_Mount_v7_tilt20_body.step/.stl  reCamera_B601_Mount_v7_tilt20_top_bar.step/.stl  assembly_preview_tilt20.png
  25    reCamera_B601_Mount_v7_tilt25_body.step/.stl  reCamera_B601_Mount_v7_tilt25_top_bar.step/.stl  assembly_preview_tilt25.png
  30    reCamera_B601_Mount_v7_tilt30_body.step/.stl  reCamera_B601_Mount_v7_tilt30_top_bar.step/.stl  assembly_preview_tilt30.png

Scripts:
  generate_recamera_mount.py [TILT]   build + exact B-rep checks + export
  render_preview.py [TILT]            preview PNG
  validate_tilt.py TILT... | v5       B-rep checks, gripper clearance,
                                      optics, FOV (v5 = v5 STEP parts)
  validation_output.txt               raw output (v5, 20, 25, 30)

Commands (from this directory):
  timeout 600 uv run --python 3.11 --with build123d --with trimesh \
    --with matplotlib --with scipy --with numpy --with manifold3d \
    --with rtree python validate_tilt.py v5 20 25 30
  (same environment for generate_recamera_mount.py and render_preview.py)

Regenerating 20/25/30 with the current generator produced byte-identical
STL files (md5 listed in validation_output.txt), so the validated geometry
is the exported geometry. No generator fix was needed; the generator's
check block was moved into brep_checks() so validate_tilt.py can reuse it.


2. Results
----------
2.1 Exact B-rep checks (OCC booleans)
                                                v5 STEP   20 deg    25 deg    30 deg
  body / top bar: one valid solid each          yes       yes       yes       yes
  clamp interface y<=0 added/lost vs ref mm3    0 / 0     0 / 0     0 / 0     0 / 0
  clamp interface y<=0 added/lost vs v5 mm3     0 / 0     0 / 0     0 / 0     0 / 0
  body, top bar vs camera envelope mm3          0         0         0         0
  body, top bar vs B3 CAN cover mm3             0         0         0         0
  top bar vs body overlap mm3                   0         0         0         0
  top bar lift 0.5/2/5/10/20 mm on tilted axis  0         0         0         0
  top bar 60 mm sweep vs body / CAN mm3         0 / 0     0 / 0     0 / 0     0 / 0
  camera insertion path (90 mm sweep) mm3       0         0         0         0
  left/right/upper 20 mm connector keep-out mm3 0         0         0         0
  retention region vs v5 in camera frame mm3    0         0         0         0
    (added and lost, region volume 21296.241 mm3; top bar symdiff 0)
  body volume mm3                               37983.8   50435.3   53113.4   55712.7
  top bar volume mm3                            4210.5    4210.5    4210.5    4210.5
  body bbox max y / max z (mount frame) mm      44.9/40.4 56.6/49.9 58.7/53.5 60.3/56.9

2.2 Gripper clearance (pla7_green, cnc7, motor_7, both fingers;
    jaw openings 0, 25, 50 mm)
                                                v5 STEP   20 deg    25 deg    30 deg
  camera envelope min clearance mm              11.563    15.889    16.021    16.120
  B3 CAN cover min clearance mm                 11.563    24.522    27.154    28.898
  body above y=0.5 min clearance mm             6.765     6.765     7.121     7.121
  top bar min clearance mm                      >30       >30       >30       >30
  gripper points inside the above objects       0         0         0         0
  whole body: gripper points inside             3383      3383      3383      3383

The 3383 gripper sample points inside the whole body are identical in v5
and all variants and lie in the clamp interface (y <= 0.5): the body above
y = 0.5 has 0 inside points. This is the reference D435 interface in its
MJCF pose against pla7_green / cnc7 / motor_7, not introduced by the tilt.
The minimum clearance was found at opening 0 in every case.

2.3 Optics (pinhole; lens = centre of the camera-envelope front face)
                                                v5 STEP   20 deg    25 deg    30 deg
  optical axis pitch below horizontal deg       0         20.000    25.000    30.000
  lens centre (X, Z) mm                         -88.1,69.1 -79.9,68.7 -78.0,68.2 -76.0,67.5
  lens to tip distance mm                       112.0     105.4     103.6     101.7
  tip angle below optical axis deg              38.11     20.68     16.17     11.58
  tip lateral angle deg                         0.30      0.26      0.26      0.26
  tip position, half vertical FOV 17 deg        2.565 OUT 1.235 OUT 0.948 IN  0.670 IN
  tip position, half vertical FOV 25 deg        1.682 OUT 0.809 IN  0.622 IN  0.439 IN
  finger front points, opening 50 mm, h/40 deg  +/-0.68   +/-0.61   +/-0.60   +/-0.60
  line of sight lens -> tip                     clear     clear     clear     clear

Tip position = tan(vertical angle) / tan(half vertical FOV), as a fraction
of the half image height below centre along the image vertical (short)
axis; 1.0 = bottom edge. h/40 = horizontal position of the front-most
finger vertex as a fraction of the half image width (long axis) for a
40 deg half horizontal FOV. Line of sight was checked against the body,
the top bar, and the gripper at openings 0, 25 and 50 mm (last 1 mm before
the tip excluded, since the tip lies on the finger surface).

2.4 FOV intrusion
Rectangular pyramid, +-25 deg vertical (short axis), +-40 deg horizontal
(long axis), apex at the lens-face centre, 250 mm long. Mesh boolean
(manifold3d) with the body and top-bar tessellations: 0 mm3 for v5, 20, 25
and 30. Closest sampled body/top-bar point in front of the lens plane:
41.6 / 41.6 / 41.6 / 39.6 deg outside the pyramid (v5 / 20 / 25 / 30).
The gripper itself was not treated as an obstruction (it is the target).

2.5 Method and resolution
- B-rep checks: exact OCC booleans on the build123d solids.
- Gripper clearance: gripper meshes sampled at 0.5 mm spacing (random
  uniform by area) plus all vertices; points pre-filtered to each object's
  bounding box + 30 mm; candidates within 2 mm of the sampled minimum
  refined with exact point-to-triangle distance against the object
  tessellation (0.05 mm linear tolerance); point-in-solid test only for
  points inside the object's bounding box. Jaw openings 0, 25, 50 mm
  (joint7 range; both fingers slide 1:1). Clearance error is bounded by
  the 0.5 mm gripper sample spacing. Gripper meshes are not watertight, so
  no exact overlap volume against the gripper is computed.
- Runtime: 137-152 s per angle.

2.6 Earlier hang
The previous validate_tilt.py used trimesh sample_surface_even at 0.4 mm
spacing (about 0.9 M points across the gripper meshes per opening set),
11 jaw openings, and ran trimesh contains() (ray-cast point-in-solid,
rtree fallback without embree, measured about 80 us per point) on every
gripper sample point for 5 objects with no bounding-box prefilter. That is
about 16 M point-in-solid tests per angle, roughly 20-25 min per angle and
over 80 min for v5 + 20 + 25 + 30.


3. Print notes
--------------
- Two parts per variant: body and top_bar. Print separately, as for v5.
- Top bar: flat 3 mm plate with ears; print flat on its large face.
- Body: orientation not determined here. The support wedge is solid
  (adds 12.5 / 15.1 / 17.7 cm3 vs the v5 body for 20 / 25 / 30 deg); it is
  not hollowed or ribbed.
- Fasteners and nuts are the same as v5: two screws through the printed
  top-bar ears into square nuts in side-open pockets (3.2 mm clearance
  holes, pockets 5.8 mm along depth, 2.6 mm high). The clamp interface and
  its fasteners are unchanged from v5. Verify screw length and nut fit
  before tightening; do not drive screws into the camera.
- Assembly as v5: remove top bar, lower the camera into the holder, insert
  the square nuts from the outer sides, attach the top bar.


4. Not validated
----------------
- Physical fit on the arm and clamp seating.
- Print tolerance, print orientation and supports.
- Real reCamera FOV, lens position inside the housing, distortion. The
  17 / 25 deg half vertical FOV and 40 deg half horizontal FOV are inputs,
  not measurements. Tilt 20 at 17 deg (1.235) and tilt 25 at 17 deg
  (0.948) are close to the image edge; small errors in lens position or
  FOV change the in/out result.
- Cable routing and connector access with the raised rear (keep-out boxes
  are checked; real cable overmolds and bend radii are not).
- Strength and stiffness of the tilted cradle, wedge and bridge, and
  retention under arm loads.
- Arm motion beyond the gripper body and jaws (other links, joint ranges).
- The camera envelope is a 40 x 40 x 36.5 mm box, not the real housing.
