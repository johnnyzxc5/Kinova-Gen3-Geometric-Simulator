# Validation and its limits

`tests/validate_kinova.m` uses 40 configurations with fixed seed13102026 and compares independent COM-Jacobian/Lagrange calculations with the optimized spatial RNEA model. It checks:

- Manufacturer-chain kinematics against the space product of exponentials.
- Twist Jacobian against pose derivatives.
- Mass and gravity against link COM Jacobians.
- Gravity against the derivative of gravitational potential.
- Coriolis forces against kinetic-energy derivatives, plus the power identity.
- Mass symmetry and positive definiteness.
- Exact stock/optimized RNEA and mass-matrix parity.
- Forward/inverse dynamics with a flange wrench and the sign convention.
- Adjoint/exponential conjugation and wrench/twist power duality.
- Passive friction, apparent-inertia conversion and model-change cache invalidation.

The independent reference uses the same parameter source. These checks establish coherent implementation of those parameters, not the accuracy of the parameters on hardware.

`run_kinova_demo` holds the control period fixed while halving the plant step. It checks trajectory differences and energy balance, and records torque utilization, velocity utilization and joint margins. These are numerical checks for the demonstration, not a stability proof or continuous-time safety certification.

`tools/benchmark_kinova` measures the same full dynamics outputs M and c+g+f for stock and optimized functions, on 100 changing states after warm-up, repeated five times. A representative run is retained in `results/benchmark.json`. Benchmark results vary with MATLAB release, CPU, JIT warm-up and concurrent load. Animation, model loading, controller computation and integration are excluded from this timing comparison.

Source provenance is reproducible: `tools/verify_sources.py` checks every bundled source and mesh against its recorded SHA-256. MATLAB runs without network access; the Python source-preparation scripts are optional development tools. `prepare_sources.py` rebuilds parameters from the bundled URDF; optional `--shape-source` and `--flatness-source` arguments re-vendor the author's kernels from local source repositories. Existing kernels remain bundled when these arguments are omitted. The parameter and mesh scripts use only the Python standard library.

`tests/test_simulation_inputs.m` additionally exercises environmental work, multiplicative effectiveness and saturated user-supplied effort. It tests zero/near-zero and half-turn geometry states and body-Jacobian consistency. Its receipt is `results/simulation_inputs_validation.json`.
