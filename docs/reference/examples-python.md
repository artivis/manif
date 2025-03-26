# Python3 Examples

The project provides some self-contained and self-explained executables implementing some real problems.
Their source code is located in [`examples/`](https://github.com/artivis/manif/tree/devel/examples) folder.

These demos are:

- [`se2_localization.py`](https://github.com/artivis/manif/tree/devel/examples/se2_localization.py): 2D robot localization based on fixed landmarks using SE2 as robot poses. This implements the example V.A in the paper.
- [`se3_localization.py`](https://github.com/artivis/manif/tree/devel/examples/se3_localization.py): 3D robot localization based on fixed landmarks using SE3 as robot poses. This re-implements the example above but in 3D.
- [`se2_sam.py`](https://github.com/artivis/manif/tree/devel/examples/se2_sam.py): 2D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE2 robot poses. This implements a the example V.B in the paper.
- [`se3_sam.py`](https://github.com/artivis/manif/tree/devel/examples/se3_sam.py): 3D smoothing and mapping (SAM) with simultaneous estimation of robot poses and landmark locations, based on SE3 robot poses. This implements a 3D version of the example V.B in the paper.
- [`se3_sam_selfcalib.py`](https://github.com/artivis/manif/tree/devel/examples/se3_sam_selfcalib.py): 3D smoothing and mapping (SAM) with self-calibration, with simultaneous estimation of robot poses, landmark locations and sensor parameters, based on SE3 robot poses. This implements a 3D version of the example V.C in the paper.
- [`se_2_3_localization.py`](https://github.com/artivis/manif/tree/devel/examples/se_2_3_localization.py): A strap down IMU model based 3D robot localization, with measurements of fixed landmarks, using SE_2_3 as extended robot poses (translation, rotation and linear velocity).

To run a demo, simply go to the `examples/` folder and run:

```bash
cd examples
python3 se2_localization.py
```
