ARUCO MARKERS FOR THE TWO ROVERS


FILES

  make_markers.py        generator, vector SVG at an exact physical size
  aruco_4x4_50mm.svg     source
  aruco_4x4_50mm.pdf     PRINT THIS - A4 portrait

Four markers on the page: ID 0 and ID 1 to use, plus a spare of each. Markers get
scuffed, bent and coffee-stained, and a damaged one fails quietly rather than
loudly, so keep the spares.


WHAT WAS CHOSEN AND WHY

  Dictionary  DICT_4X4_50
  IDs         0 and 1
  Size        50 mm side, INCLUDING the black border
  Cell        8.333 mm (a 4x4 marker is 6x6 cells: 4x4 of data plus a 1-cell
              black border all round)

4x4 gives the biggest cells for a given physical size, and cell size is what buys
detection range and blur tolerance. The usual complaint about 4x4 is weaker
separation between codes, so more false positives - fixed by whitelisting IDs 0
and 1 and rejecting everything else.

Verified: all four markers on the generated page detect, with correct IDs.


PRINTING

1. Print aruco_4x4_50mm.pdf at 100% / "actual size". Fit-to-page OFF.
2. Measure the 100 mm reference line. If it is not exactly 100 mm the printer
   scaled the page, and every distance you measure with these markers will be
   wrong by the same ratio.
3. Matte paper, not glossy. Glare kills detection, and it already ruined two
   rounds of stereo calibration images on this project.
4. Cut on the grey line. KEEP THE WHOLE WHITE MARGIN - that is the quiet zone,
   and the detector needs it to find the marker's outer edge. Trimming flush to
   the black square is the single most common reason detection silently fails.
5. Measure a printed marker edge to edge, black border included. Use THAT number
   in the pose code, not 50.


MOUNTING

  - Front face of each rover. Both events are front-to-front: the rovers face
    each other at the start, and meet head-on on the far side. So one marker per
    rover covers both.
  - At camera height, so the marker sits in the middle of the frame rather than
    the distorted edge.
  - Flat and rigid. Glue it to card or plastic. A curved marker returns a wrong
    pose, and here the pose IS the measurement.
  - Perpendicular to the direction of approach, facing straight out.
  - If the front panel has no room, put it on a short mast. Vertical space is
    usually easier to find than footprint, and a mast also solves the
    camera-height requirement.


RANGE

At full resolution (fx 1137.6), a 50 mm marker spans:

    0.3 m -> 190 px      1.0 m ->  57 px
    0.5 m -> 114 px      1.5 m ->  38 px
    2.0 m ->  28 px, about the limit

Detection needs roughly 4-5 px per cell, so this is solid to about 1.5-2 m.

Run the detector on the FULL RESOLUTION frame, not the 2x-downscaled one used for
SGBM. Marker detection is cheap next to stereo matching, and full res doubles the
range for nothing.

Short range is fine, because the marker is not what stops the rovers - the sonars
are. Each rover sees the other as an obstacle under 1 m and brakes. The marker
only has to say "that obstacle is rover 1" and then give the measurement, and the
measurement is taken stationary at roughly half a metre, where the marker spans
114 px.


DETECTING WHILE MOVING VERSUS MEASURING WHILE STOPPED

Two different jobs, with different requirements:

  Detect  - happens while approaching, needs robustness not accuracy. Rough is
            fine; it only has to fire.
  Measure - happens after both rovers have stopped. No motion blur, average
            several frames, and this is the number that goes into the map.

Do not take the measurement while moving. At a 33 ms exposure and 0.4 m/s closing
speed there is real blur, and a 50 mm marker has little margin to spare.


GETTING THE DISTANCE - USE BOTH CAMERAS

Do NOT use single-camera solvePnP. It is adequate for position but noisy and
ambiguous for orientation: a near-frontal planar marker has two nearly equal pose
solutions, so the estimated angle flips between frames. It is a well known ArUco
failure mode.

Instead detect the marker in both rectified eyes and triangulate its four corners.
The stereo rig is calibrated with the baseline known to 0.5 percent, so the corners
come out in real 3D with correct scale and no flip ambiguity.

    import cv2, numpy as np, yaml

    def load(p):
        d = yaml.safe_load(open(p))
        g = lambda k, r, c: np.array(d[k]['data'], float).reshape(r, c)
        return (g('camera_matrix', 3, 3), g('distortion_coefficients', 1, 5),
                g('rectification_matrix', 3, 3), g('projection_matrix', 3, 4))

    KL, DL, R1, P1 = load('stereo_calibration/out/1920x1200/acamst_left.yaml')
    KR, DR, R2, P2 = load('stereo_calibration/out/1920x1200/acamst_right.yaml')
    SIZE = (1920, 1200)

    mLx, mLy = cv2.initUndistortRectifyMap(KL, DL, R1, P1, SIZE, cv2.CV_32FC1)
    mRx, mRy = cv2.initUndistortRectifyMap(KR, DR, R2, P2, SIZE, cv2.CV_32FC1)

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(dic, cv2.aruco.DetectorParameters())
    ALLOWED = {0, 1}
    MARKER_MM = 50.0          # replace with the MEASURED printed size

    def markers_3d(left_img, right_img):
        L = cv2.remap(left_img, mLx, mLy, cv2.INTER_LINEAR)
        R = cv2.remap(right_img, mRx, mRy, cv2.INTER_LINEAR)
        cL, iL, _ = det.detectMarkers(L)
        cR, iR, _ = det.detectMarkers(R)
        if iL is None or iR is None:
            return {}
        dL = {int(i): c.reshape(4, 2) for c, i in zip(cL, iL.ravel())
              if int(i) in ALLOWED}
        dR = {int(i): c.reshape(4, 2) for c, i in zip(cR, iR.ravel())
              if int(i) in ALLOWED}
        out = {}
        for mid in dL.keys() & dR.keys():
            h = cv2.triangulatePoints(P1, P2,
                                      dL[mid].T.astype(float),
                                      dR[mid].T.astype(float))
            out[mid] = (h[:3] / h[3]).T        # 4 corners, metres, left cam frame
        return out

Distance to a marker is then the length of its centroid vector:

    corners = markers_3d(L, R)[0]
    distance_m = float(np.linalg.norm(corners.mean(axis=0)))


A FREE SELF-CHECK, USE IT

The four triangulated corners should form a square whose sides equal the printed
marker size. So measure them:

    sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]

If those are not all close to MARKER_MM / 1000, the reading is bad - a
misdetection, a bent marker, a bad rectification, or a scaled print. Reject the
reading rather than feeding it into the map.

This is the cheapest validity test available and it costs four subtractions. A
wrong start distance silently offsets both maps for the whole run, so it is worth
checking every single time.


GET THE HEADING FROM THE WALL, NOT THE MARKER

A 50 mm marker gives good distance but weak orientation: the four corners span
only 50 mm, so there is very little lever arm to estimate the plane's angle from.
Orientation error is also the worse of the two here - a small rotation error makes
the two maps splay apart, growing with distance from the origin, while a
translation error just offsets them.

Both rovers start against the same wall, so each can measure that wall's
direction from its own sonars or stereo depth. That fixes their relative heading
far better than a 50 mm marker can.

    marker  -> distance between the rovers
    wall    -> relative heading

Average several frames while stationary for both.


WHEN IT FAILS

  no detection at all       quiet zone trimmed off, or glossy paper glare
  detects then drops out    motion blur; stop before measuring
  wrong distance            print scale, or MARKER_MM not the measured value
  jittery orientation       expected at 50 mm; use the wall for heading
  detects a phantom marker  whitelist IDs 0 and 1
  rovers stop but see no marker
                            needs a fallback, or both sit there thinking they
                            have hit a wall


REGENERATING

    python3 make_markers.py --size 40
    python3 make_markers.py --size 50 --ids 0,1,2,3 --no-spares

The generator refuses a quiet zone under one cell, and refuses a layout that will
not fit on A4.


MEASURED ON THE CAMERA - 2026-08-17

A printed marker was held in front of the stereo pair and detected as id 1 from
DICT_4X4_50. Detection cost 4-10 ms per frame at 960x600, so running it on every
frame is essentially free.

TWO INDEPENDENT RANGES, and comparing them is the useful part:

  solvePnP on the four corners, from the known printed size
  stereo depth over the marker area

  assumed size    PnP        stereo     differ
  36 mm         0.427 m     0.749 m     +322 mm   (numDisparities=128)
  36 mm         0.424 m     0.576 m     +152 mm   (numDisparities=256)

TWO LESSONS, both of which produce plausible-looking wrong numbers.

  THE DISPARITY SEARCH RANGE SETS A MINIMUM DEPTH. depth = 87.12/disparity, so
  numDisparities=128 cannot see closer than 0.68 m. A marker at 0.43 m then
  reports 0.749 m: not a measurement, the floor of the range, and it looks
  entirely reasonable. Widening to 256 moved it to 0.576 m.

  PnP SCALES LINEARLY WITH THE ASSUMED MARKER SIZE, so getting that wrong is
  invisible in the reprojection error. PnP was steady at 0.427 and 0.424 across
  runs, so it is not noisy -- it is consistently scaled by whatever size it was
  told. Back-solving from the stereo range gives an implied black-square edge of
  36 x 0.576/0.424 = 48.9 mm, which matches the 50 mm marker in this directory
  rather than the 36 mm one. 0.424 x 50/36 = 0.589 m, agreeing with stereo to
  13 mm.

SO: MEASURE THE PRINTED BLACK SQUARE WITH A RULER and pass that size. It is the
same trap as the checkerboard square earlier in this project, where a ruler
reading of 25.65 mm turned a confusing 8% disagreement into agreement at 0.3 mm.

Whatever consumes markers later -- rover-to-rover localisation -- should carry the
measured edge length as a parameter, not a constant, and should cross-check PnP
against stereo rather than trusting either alone.
