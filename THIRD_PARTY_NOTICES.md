# Source attribution and licenses

## New code and author-owned reused kernels

The new `+kinova` package, examples, tests and utilities are licensed under the repository MIT license, copyright2026 Johnny Garcia. The SHAPE kernels and Flatness-MPC quintic phase functions were reused at the author's request and are covered by the selected MIT license for this new repository. Original function content is preserved; local source paths and SHA-256 hashes are in `sources/manifest.json`.

## Modern Robotics

`third_party/modern_robotics` contains the local Modern Robotics MATLAB functions used by SHAPE. Their original comments are retained and the NxRLab MIT license is included in that directory. InverseDynamics is the local scientific copy, which differs from the upstream text; do not claim that every file is an unmodified current upstream version. The remaining geometric/reference functions are the preserved local copies, with hashes recorded.

Source: https://github.com/NxRLab/ModernRobotics. Copyright2019 NxRLab. See `third_party/modern_robotics/LICENSE`.

## Kinova model and visual geometry

`sources/kinova_gen3_6dof_novision.urdf`, its derived geometric/link parameters and `sources/meshes` come from Kinovarobotics/ros_kortex at commit `5f66bbd1be84cb699020621dc7c8f3df01394960`. They retain Kinova's BSD-style notice; they are not relicensed under the new-code MIT license. The full original notice is preserved in `sources/kinova_license.txt`. Exact URLs and hashes are in the source and mesh manifests.

Copyright2018 Kinova inc. The robot name identifies the modeled platform; it does not imply endorsement by Kinova.

## Parameter literature

The actuator proxy constants are attributed to Adrien Cholet's2025 master's thesis, Sec.3.1.3/Sec.4.1 and Tables A.1/B.1. The viscous/Coulomb parameter values are attributed to Bastien Muraccioli, Mathieu Celerier, Mehdi Benallegue and Gentiane Venture, RSS2025, DOI10.15607/RSS.2025.XXI.084. Original papers and thesis PDFs are not bundled or relicensed; the numerical parameter transcription and its modeling assumptions are documented in `docs/MODEL.md`.

No MATLAB, Simulink, toolboxes, generated binaries, or commercial software distribution is bundled.
