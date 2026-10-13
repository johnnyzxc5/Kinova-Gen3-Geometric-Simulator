# Geometric mechanical model

## Frames and Lie algebra

The base is fixed. A pose `T_ab` maps coordinates from body frame b to frame a. Twists are `[omega; v]`; wrenches are `[moment; force]`. `Ad(T)` transforms twists, and the dual map transforms wrenches so their power pairing is invariant. URDF roll-pitch-yaw is composed as `Rz(yaw)*Ry(pitch)*Rx(roll)`.

The home pose of each link is obtained by accumulating relative URDF joint poses. A joint axis a in its child frame becomes w = R_home*a in the base frame; its space screw is `[w; -cross(w,p_home)]`. The flange motion is the product of exponentials `exp([S1]q1)...exp([S6]q6)*T_home`. Body screws are `Ad(inv(T_home))*Slist`.

For recursive dynamics, `Mlist(:,:,i)` is the home pose of frame i relative to frame i-1, not the pose relative to the base. `home_links` stores the latter separately. Body frames need not lie at centers of mass if the entire spatial inertia, screw axes and transforms use the same origin.

## Spatial inertia and energy

For a link-local center-of-mass displacement c, let C = skew(c). The spatial inertia about the link origin is

```text
G = [I_com - m*C*C,  m*C;
                 -m*C, m*eye(3)].
```

The rotational tensor in the inertial URDF frame is rotated into the link frame first. This uses the parallel-axis theorem without losing the cross blocks. Adding the rotor mass again to a URDF link would risk double-counting the manufacturer link assembly; this model adds only the approximate joint apparent-inertia term described below.

The Hamiltonian is `H = 0.5*qd'*M(q)*qd - sum(m_i*gravity'*p_com_i)`. The additive potential reference is arbitrary. The model is mechanical: the port input tau and flow qd are joint-side quantities.

## Apparent inertia

Cholet Table A.1 assigns Medium actuators to joints1-3 and Small actuators to joints4-6. Table B.1 gives:

| Parameter | Medium | Small |
|---|---:|---:|
| Motor inertia, kg m² | 9.3e-6 | 5.6e-6 |
| Reducer output inertia, kg m² | 7.9e-6 | 3.3e-6 |
| Reduction N | 100 | 100 |
| Apparent inertia N² Jm + Jr, kg m² | 0.0930079 | 0.0560033 |

This follows the output-reference convention declared in Cholet Sec.3.1.3 and the kinetic-energy argument of Modern Robotics Sec.8.9.2. An inertia already expressed at the reducer output is not multiplied by N² again.

Cholet Sec.4.1 identifies these as commercially available substitutes selected to approximate torque/speed performance, not experimentally identified internal Kinova actuators. Their usage here is a documented proxy. The constant diagonal approximation neglects the additional rotor-body motion and reactions of the more complete formulation in Modern Robotics Sec.8.9.3. That fuller model is outside this release's scope.

No efficiency constant multiplies the mass matrix. An ideal gearing energy model is used, with losses lumped into the joint friction law. This avoids mixing a motor-side command convention with joint-side generalized effort or charging the same loss twice.

## Friction

The RSS paper identifies Coulomb/viscous parameters on a seven-joint Gen3: four large actuators and three small. The six-joint URDF arm uses the transferred grouping three large/three small. This transfer is an assumption, not a new six-joint calibration.

Default `f(qd) = Fv.*qd + Fc.*tanh(kappa*qd)` with kappa=50 s/rad. Coefficients are Fv=[2,2,2,1.36,1.36,1.36] N m s/rad and Fc=[2.15,2.15,2.15,1.60,1.60,1.60] N m. Every component satisfies `qd_i*f_i >= 0`; the law is smooth and zero at rest. The smoothing parameter is our numerical regularization. This does not create a threshold torque or set-valued static friction at zero velocity.

The optional `shape_blend` multiplies the default expression by `tanh(abs(kappa*qd))`, matching the SHAPE plant's velocity-only branch. Acceleration-dependent friction precompensation is not a physical plant law here.

## Controller and integrator

The demo maps a quintic joint reference through `tau = ID(q,qd,qdd_ref + Kp*(qref-q) + Kd*(qdref-qd))`. This resembles the flat-output inverse mapping used in the earlier projects, but contains no MPC optimization. Command saturation occurs before optional multiplicative actuator effectiveness. Commands are held for one control period while the plant is integrated by RK4 substeps.

Optional environmental wrench is defined in the flange body frame and adds `J_body'*F_env`. In inverse dynamics, Modern Robotics Ftip is instead the wrench the robot applies to the environment: use its negative to describe F_env. The forward/inverse tests check this sign convention.

The integrated powers use the same RK4 stage states as the mechanical plant: joint input, environment input and positive friction dissipation. The checked residual is

```text
H(t)-H(0) - integral(tau_applied'*qd)
           - integral(F_env'*J_body*qd) + integral(f(qd)'*qd).
```

An optional effectiveness function and external wrench function are evaluated at each RK4 stage. End-effector contacts, collisions, elastic joints, electrical dynamics and sensor models are outside the current scope. Manufacturer joint limits are monitored at stages; they are not implemented as hard stops. The tested step size is for the default apparent-inertia/friction model. A rigid-links-only or user-specified small-inertia model may be substantially stiffer and requires a smaller step or an appropriate stiff solver.
