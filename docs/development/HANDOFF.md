# Kinova Gen3 — Global Geometric Dynamics on (S¹)⁶: Protocolo Handoff

**Archivo de seguimiento y transferencia entre modelos de IA (Flash 3.8 High y Argon/Astra)**  
**Repositorio:** `C:\Users\Johnny\Desktop\Git\Kinova-Gen3-Geometric-Simulator`  
**Rama activa:** `experiment/kinova-s1-dynamics`  
**Última actualización:** 27 de octubre de 2026 (Hito H0 completado, arrancando H1)

---

## 1. Calendario de Hitos

| Hito | Trabajo | Modelo Sugerido | Estado | Fecha / Nota |
|:---:|---|:---:|:---:|---|
| **H0** | **Auditoría y baseline** | **Flash 3.8 High** | **COMPLETADO** | 27-Oct-2026. Auditoría completa, tests ejecutados en MATLAB R2026a, baseline 6.75x y 3.26e-9 J documentado en `AUDIT_BASELINE.md`, rama `experiment/kinova-s1-dynamics` creada. |
| **H1** | **Adaptación e implementación S¹** | **Flash 3.8 High** | **COMPLETADO** | 27-Oct-2026. Paquete `+kinova_s1`, kernel `rnea_shared_kinematics_S1`, manejo de juntas continuas/acotadas, paridad matemática de máquina verificada, `GEOMETRIC_DYNAMICS_IMPLEMENTATION.md` y `MATHEMATICAL_FORMULATION.md`. |
| **H2** | **Tests, simulaciones y benchmarks** | **Flash 3.8 High** | **EN PROGRESO** | Suites de tests unitarios y de consistencia, comparación A/B/B-alg, paridad física y benchmarks. Al finalizar H2, **DETENERSE para Argon**. |
| **H3** | **Revisión matemática independiente** | **Argon / Astra** | **PENDIENTE** | Auditoría matemática y topológica independiente. Al finalizar H3, **DETENERSE para Flash**. |
| **H4** | **Correcciones, GIFs y documentación** | **Flash 3.8 High** | **PENDIENTE** | Cierre de puntos de H3, generación de animaciones GIF lado a lado y portada `README.md`. |
| **H5** | **Publicación, revisión visual y cierre** | **Flash 3.8 High** | **PENDIENTE** | Merge a `main`, push remoto, verificación de renderizado en GitHub y entrega final. |

---

## 2. Registro de Hitos

### Hito H0 — Auditoría y Baseline
* **Estado:** **COMPLETADO**
* **Responsable:** Flash 3.8 High
* **Resultados clave:**
  - Código fuente original (+kinova) auditado y validado íntegramente.
  - Tag base reproducible `v0.1.0` confirmado en commit `e2aefe2`.
  - Rama `experiment/kinova-s1-dynamics` creada.
  - Baseline en MATLAB R2026a Update 4:
    - `validate_kinova`: 40/40 estados PASS. Todos los errores numéricos en precisión de máquina ($\le 2.5 \times 10^{-15}$).
    - `test_simulation_inputs`: PASS. Error de energía forzada $6.85 \times 10^{-9}\text{ J}$.
    - `benchmark_kinova`: Modern Robotics canónico 981.40 µs vs RNEA optimizado 145.40 µs (aceleración 6.75x).
    - `run_kinova_demo`: Error de balance energético nominal 3.258e-9 J, paso fino 2.727e-9 J.
  - Documento `docs/development/AUDIT_BASELINE.md` redactado y archivado.

---

### Hito H1 — Adaptación e Implementación S¹
* **Estado:** **COMPLETADO**
* **Responsable:** Flash 3.8 High
* **Resultados clave:**
  - Implementación completa del paquete `+kinova_s1`:
    - `model.m`: clasificación formal de juntas continuas (1, 4, 6) y acotadas (2, 3, 5), cálculo de arcos $u_i = (1, 0)^T$, verificación de ejes puros en $\hat{z}$ ($A_i = [0;0;1;0;0;0]$).
    - `config_matrix.m`: validación y normalización $X \in (S^1)^6 \subset \mathbb{R}^{2\times 6}$.
    - `from_q.m` y `to_q.m`: levantamiento centrado seguro de cortes de rama para juntas acotadas, y desenrollado continuo con preservación de vueltas acumuladas para juntas continuas.
    - `exp_s1.m`: acción del grupo de Lie $SO(2)^6$ sobre $(S^1)^6$ con preservación de norma unitaria a nivel de máquina ($2.22 \times 10^{-16}$).
    - `screw_exp.m`: mapa exponencial algebraico en $SE(3)$ según Teorema 2.1, sin funciones trascendentales en tornillos revolutos unitarios de paso cero.
    - `check_limits.m`: verificación rigurosa de límites de arco para 2, 3, 5 y admisión completa sin topes inventados para 1, 4, 6.
    - `box_diff.m` y `geodesic_diff.m`: desplazamientos conexos en arcos acotados y geodésicos en factores continuos con soporte de vueltas enteras.
    - `reference.m`: generador de trayectoria quintic intrínseca sobre $(S^1)^6$.
    - `fk.m` y `frames.m`: cinemática directa PoE y cadena de eslabones independientes.
    - `jacobian.m`: jacobianos de twist espacial y corporal directamente en $(S^1)^6$.
    - `dynamics.m`, `inverse_dynamics.m`, `forward_dynamics.m`, `energy.m`: dinámica RNEA optimizada con inercia aparente $J_{\text{app}}$ (Cholet, rigid, custom) y fricción (Muraccioli, blend, none) respetadas al 100%.
    - `lagrange_reference.m`: oráculo independiente COM-Jacobiano en $(S^1)^6$ para validación cruzada.
    - `simulate.m`: integrador Munthe-Kaas RK4 en $(S^1)^6 \times \mathbb{R}^6$ con ZOH, saturación de torque, efectividad, wrenches externos y telemetría desenrollada.
    - `draw_robot.m`: renderizado 3D de eslabones oficiales Kinova desde configuración $X$.
  - Kernel optimizado: `third_party/shape_kernels/rnea_shared_kinematics_S1.m` adaptado para 6 DoF con homomorfismo adjunto por rotación plana de filas y caché bitwise `uint64`.
  - Smoke tests verificados en MATLAB R2026a:
    - Error FK frente a Versión A: $1.18 \times 10^{-15}$
    - Error Matriz de Masa $M$: $5.84 \times 10^{-16}$
    - Error Bias dinámico $b$: $3.35 \times 10^{-15}$
    - Error de norma unitaria en simulación: $2.22 \times 10^{-16}$
    - Paridad de trayectoria en simulación: $5.86 \times 10^{-11}\text{ rad}$
    - Error de balance de energía: $3.258 \times 10^{-9}\text{ J}$
  - Documentos técnicos `GEOMETRIC_DYNAMICS_IMPLEMENTATION.md` y `MATHEMATICAL_FORMULATION.md` redactados.

---

## 3. Decisiones Matemáticas y de Arquitectura (Hito H1)

### 3.1. Variedad de Configuración y Estado
- Configuración: $X \in (S^1)^6$, representada computacionalmente como una matriz $2 \times 6$ donde la columna $i$ es $x_i = [c_i; s_i] = [\cos q_i; \sin q_i]$ satisfaciendo $c_i^2 + s_i^2 = 1$.
- Velocidad: vector euclidiano de 6 velocidades angulares independientes $\omega \in \mathbb{R}^6$ (álgebra de Lie $\mathfrak{so}(2)^6 \cong \mathbb{R}^6$).
- Espacio de fases: $T(S^1)^6 \cong (S^1)^6 \times \mathbb{R}^6$.

### 3.2. Tratamiento de Juntas: Continuas vs. Acotadas
- **Juntas Continuas ($i \in \{1, 4, 6\}$):**
  - No poseen topes mecánicos en el URDF ($q_{\min} = -\infty, q_{\max} = +\infty$).
  - La condición de admisibilidad en `check_limits` se reduce estrictamente a la pertenencia a la variedad: $|c_i^2 + s_i^2 - 1| \le \text{tol}_{\text{norm}}$.
  - Su margen a los topes es $+\infty$ (`inf`).
  - **Vueltas acumuladas (Winding):** La orientación física en $S^1$ es intrínsecamente periódica ($q \sim q + 2\pi k$). Sin embargo, para trayectorias de control y telemetría, una función `to_q(X, m, q_prev)` implementará desenrollado de fase cuando se suministre el estado previo $q_{\text{prev}}$. Si no se suministra, devolverá la rama principal en $(-\pi, \pi]$.
- **Juntas Acotadas ($i \in \{2, 3, 5\}$):**
  - Poseen límites mecánicos simétricos: J2 $[-2.24, 2.24]$, J3 $[-2.57, 2.57]$, J5 $[-2.09, 2.09]\text{ rad}$.
  - Punto medio $m_i = 0$, semiamplitud $h_i = q_{\max,i} < \pi$.
  - Vector unitario central $u_i = [1; 0]$.
  - Condición de admisibilidad exacta: $u_i^T \hat{x}_i \ge \cos(h_i) - \text{tol}$ y margen $\mu_i = h_i - |\delta_i| \ge -\text{tol}$, donde $\delta_i = \text{atan2}(s_i, c_i) \in (-h_i, h_i)$.

### 3.3. Homomorfismo Adjunto y Álgebra de Lie
- Todos los ejes locales satisfacen $\mathcal{A}_i = [0;0;1;0;0;0]^T$.
- Por el homomorfismo del adjunto:
  $$\text{Ad}_{T_{i,i-1}} = \text{Ad}_{e^{-[\mathcal{A}_i]q_i}} \text{Ad}_{M_i^{-1}} = \text{diag}(R_z(x_i)^T, R_z(x_i)^T) \cdot \text{Ad}_{M_i^{-1}}$$
  Esto permite calcular $\text{Ad}_{T_{i,i-1}}$ de forma 100% algebraica a partir de $(c_i, s_i)$ y la constante precalculada $\text{Ad}_{M_i^{-1}}$, sin evaluar funciones trigonométricas ni exponenciales de matriz.

### 3.4. Inercia Aparente y Fricción
- Se conservan estrictamente los tres perfiles de inercia aparente (`cholet_apparent`, `rigid_links`, `custom`) y los tres de fricción (`muraccioli_smooth`, `shape_blend`, `none`).
- $J_{\text{app}}$ interviene exactamente una vez en $M$, dinámica inversa, dinámica directa y energía mecánica.

---

## 4. Archivos Creados / Modificados en esta Rama

- `docs/development/AUDIT_BASELINE.md`: Creado (H0).
- `docs/development/HANDOFF.md`: Creado (H0/H1).
