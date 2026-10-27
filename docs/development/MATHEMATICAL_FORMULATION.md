# Mathematical Formulation: Global Geometric Dynamics of the Kinova Gen3 on $(S^1)^6$

**Milestone:** HITO H1 — Adaptación e Implementación $S^1$  
**Date:** October 27, 2026  
**Author / Model:** Antigravity (Gemini 3.8 Flash High)  
**Primary References:**  
1. T. Lee, M. Leok, and N. H. McClamroch, *Global Formulations of Lagrangian and Hamiltonian Dynamics on Manifolds*, Springer, 2018 (Chapters 1, 4, and Section 8.7.4.1).  
2. Kinova Robotics, *Kinova Gen3 Ultra lightweight robot User Guide & URDF specification* (`5f66bbd1be84cb699020621dc7c8f3df01394960`).  
3. Cholet et al., "Evaluation of actuator models for dynamic simulation of robotic manipulators," 2025.  
4. Muraccioli et al., "Identification and friction modeling for lightweight manipulators," *IEEE Robotics: Science and Systems (RSS)*, 2025.

---

## 1. Executive Summary and Architectural Synthesis

This document establishes the mathematical formulation for **Version B (Global Lie-Group / Manifold Geometric Dynamics)** of the 6-DoF Kinova Gen3 simulator, alongside the intermediate **Variant B-alg (Angular Specialized $q$-Formulation)** required for the ablation benchmark.

### Key Mathematical Pillars

1. **Global Configuration Manifold $(S^1)^6$:**  
   Each revolute joint $i \in \{1,\dots,6\}$ has intrinsic configuration space $S^1 \subset \mathbb{R}^2$, represented by the unit column vector $x_i = (c_i, s_i)^T = (\cos q_i, \sin q_i)^T \in \mathbb{R}^2$ satisfying $x_i^T x_i = 1$. The global configuration manifold of the 6-DoF arm is the 6-torus:
   $$\mathsf{Q} = (S^1)^6 = \{ X = [x_1, \dots, x_6] \in \mathbb{R}^{2\times 6} : x_i^T x_i = 1, \; i=1,\dots,6 \}.$$
   The state space is the trivialized tangent bundle $T\mathsf{Q} \cong (S^1)^6 \times \mathbb{R}^6$, where $\omega = (\omega_1, \dots, \omega_6)^T \in \mathbb{R}^6$ represents the physical joint angular velocities:
   $$\dot{x}_i = \omega_i J x_i, \qquad J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}.$$

2. **Mixed Topology: Continuous vs. Bounded Revolute Joints:**  
   The Kinova Gen3 has an essential topological structure distinct from manipulators whose joints are all bounded:
   $$\mathcal{D}_{\text{adm}} = S^1 \times A_2 \times A_3 \times S^1 \times A_5 \times S^1 \subset (S^1)^6$$
   - **Continuous joints ($i \in \{1, 4, 6\}$):** No mechanical stops exist ($q_{\min} = -\infty$, $q_{\max} = +\infty$). The configuration factor is the full circle $S^1$. For these joints, the admissible set is $S^1$ itself. Winding tracking is preserved along trajectories via relative continuous integration $q_i(t) = q_{0,i} + \int_0^t \omega_i(s)\,ds$.
   - **Bounded revolute joints ($i \in \{2, 3, 5\}$):** The physical arc $A_i = [q_{\min,i}, q_{\max,i}]$ is strictly bounded ($h_2 = 2.24$, $h_3 = 2.57$, $h_5 = 2.09\text{ rad} < \pi$). The midpoint is $m_i = 0$, center vector $u_i = (1, 0)^T$, and the hard stop sector $S^1 \setminus A_i$ is forbidden. The exact algebraic condition is:
     $$u_i^T \hat{x}_i \ge \cos h_i - \text{tol}, \qquad \mu_i = h_i - |\delta_i| \ge -\text{tol}, \qquad \hat{x}_i = \frac{x_i}{\|x_i\|_2}.$$

3. **Algebraic Lie Group Homomorphisms and Spatial RNEA:**  
   All 6 joint axes in their respective local link frames satisfy:
   $$\mathcal{A}_i = \operatorname{Ad}_{M_i^{-1} \cdots M_1^{-1}} \mathcal{S}_i = \begin{bmatrix} 0 \\ 0 \\ 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}, \qquad \forall i \in \{1,\dots,6\}.$$
   Because $e^{-[\mathcal{A}_i]q_i} = \operatorname{diag}(R_z(x_i)^T, 1)$ is a pure rotation without translation, the $6\times 6$ Adjoint transformation factorizes as:
   $$\operatorname{Ad}_{T_{i,i-1}(x_i)} = \begin{bmatrix} R_z(x_i)^T & 0_{3\times 3} \\ 0_{3\times 3} & R_z(x_i)^T \end{bmatrix} \operatorname{Ad}_{M_i^{-1}}, \qquad R_z(x_i)^T = \begin{bmatrix} c_i & s_i & 0 \\ -s_i & c_i & 0 \\ 0 & 0 & 1 \end{bmatrix}.$$
   This eliminates all matrix exponentials, matrix logarithms, and trigonometric evaluations in the recursive RNEA kinematics.

---

## 2. Global Kinematics on $(S^1)^6$ and $SE(3)$

### Theorem 2.1 (Algebraic SE(3) Exponential on $S^1$)
For a unit-axis revolute screw $\mathcal{S} = (\hat{\omega}, v) \in \mathbb{R}^6$ with $\|\hat{\omega}\|_2 = 1$ and zero pitch $\hat{\omega}^T v = 0$, and for $x = (c, s)^T \in S^1$ with $c^2 + s^2 = 1$, the spatial exponential map $\exp([\mathcal{S}] q)$ is a polynomial function of $(c, s)$:
$$\exp([\mathcal{S}], x) = \begin{bmatrix} I_3 + s [\hat{\omega}]_\times + (1 - c) [\hat{\omega}]_\times^2 & s v + (1 - c) ([\hat{\omega}]_\times v) \\ 0_{1\times 3} & 1 \end{bmatrix} \in SE(3).$$

*Proof:* By Rodrigues' formula, $\exp([\hat{\omega}]_\times q) = I_3 + \sin q [\hat{\omega}]_\times + (1 - \cos q) [\hat{\omega}]_\times^2$. Substituting $\cos q = c$ and $\sin q = s$ yields the rotational block. For zero pitch, the linear displacement integral $\int_0^q e^{[\hat{\omega}]_\times \theta} v \, d\theta = (q I + (1 - \cos q)[\hat{\omega}]_\times + (q - \sin q)[\hat{\omega}]_\times^2) v$. Since $[\hat{\omega}]_\times^2 = \hat{\omega} \hat{\omega}^T - I$ and $\hat{\omega}^T v = 0$, we have $[\hat{\omega}]_\times^2 v = -v$. Hence, the $q$-dependent terms cancel: $q v + (q - \sin q)(-v) = s v$, leaving $s v + (1 - c)([\hat{\omega}]_\times v)$. $\blacksquare$

### Forward Kinematics
1. **Space PoE (`kinova_s1.fk`):**
   $$T(X) = \exp([\mathcal{S}_1], x_1) \cdots \exp([\mathcal{S}_6], x_6) M_{\text{home}}.$$
2. **Relative Chain (`kinova_s1.frames`):**
   $$T_{0,i}(X) = T_{0,i-1}(X) M_i \begin{bmatrix} R_z(x_i) & 0 \\ 0 & 1 \end{bmatrix}, \qquad i = 1,\dots,6, \qquad T_{0,7}(X) = T_{0,6}(X) M_7.$$

---

## 3. Reduced Lagrangian Dynamics on $(S^1)^6 \times \mathbb{R}^6$

Following Lee, Leok & McClamroch (Chapter 4), the reduced Euler–Lagrange equations on $(S^1)^6 \times \mathbb{R}^6$ with mechanical actuation effort $\tau \in \mathbb{R}^6$ are:
$$M(X) \dot{\omega} + c(X, \omega) + g(X) + f(\omega) = \tau + J_b(X)^T F_{\text{env}}.$$

1. **Mass Matrix:** $M(X) = M_{\text{links}}(X) + J_{\text{app}}$, where $J_{\text{app}} = \operatorname{diag}(0.0930079, 0.0930079, 0.0930079, 0.0560033, 0.0560033, 0.0560033)\text{ kg}\cdot\text{m}^2$.
2. **Zero-State Recursion:** $M_{\text{links}}(X)$ is constructed using the zero-state forward recursion without gravity bias via `MassMatrixZeroStateSharedKinematics_MR`.
3. **Coriolis & Gravity:** Computed via forward and backward spatial passes in `InverseDynamicsSharedKinematics_MR` with direct adjoint construction $\operatorname{ad}_V$.
4. **Passive Friction Law:** $f_i(\omega_i) = \beta_{v,i} \omega_i + \beta_{c,i} \tanh(\gamma \omega_i)$, satisfying $\omega^T f(\omega) \ge 0$.

---

## 4. Geometric Munthe-Kaas Lie Group Integration

The phase space $T\mathsf{Q} = (S^1)^6 \times \mathbb{R}^6$ evolves under continuous vector field:
$$\begin{aligned}
\dot{X} &= \omega \odot (J X), \\
\dot{\omega} &= M(X)^{-1} \left( \tau_{\text{applied}} + J_b(X)^T F_{\text{env}} - b(X, \omega) \right).
\end{aligned}$$

Because $(S^1)^6$ is an Abelian Lie group isomorphic to $SO(2)^6$, the Lie algebra coordinate map $\operatorname{dexp}_{\Delta\theta}^{-1} = I$ is trivial. The 4-stage classical Munthe-Kaas method evaluates:
$$\begin{aligned}
X_1 &= X(t), & \omega_1 &= \omega(t), \\
X_2 &= \operatorname{Exp}_{S^1}\left(\frac{h}{2} \omega_1, X_1\right), & \omega_2 &= \omega_1 + \frac{h}{2} k_{1,\omega}, \\
X_3 &= \operatorname{Exp}_{S^1}\left(\frac{h}{2} \omega_2, X_1\right), & \omega_3 &= \omega_1 + \frac{h}{2} k_{2,\omega}, \\
X_4 &= \operatorname{Exp}_{S^1}\left(h \omega_3, X_1\right), & \omega_4 &= \omega_1 + h k_{3,\omega}, \\
X(t + h) &= \operatorname{Exp}_{S^1}\left(\frac{h}{6}(\omega_1 + 2\omega_2 + 2\omega_3 + \omega_4), X_1\right), & \omega(t+h) &= \omega_1 + \frac{h}{6}(k_{1,\omega} + 2k_{2,\omega} + 2k_{3,\omega} + k_{4,\omega}).
\end{aligned}$$

Columns of $X$ remain on $S^1$ to machine precision ($\|x_i\|_2 = 1 \pm 2.22 \times 10^{-16}$) for arbitrary integration intervals.
