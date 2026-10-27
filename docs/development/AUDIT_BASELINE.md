# Auditoría Técnica y Baseline del Kinova Gen3 Geometric Simulator

**Fecha:** 27 de octubre de 2026  
**Commit base:** `e2aefe2` (`v0.1.0`, `main`)  
**Rama de trabajo:** `experiment/kinova-s1-dynamics`  
**Entorno validado:** MATLAB R2026a Update 4 (`26.1.0.3312084`), Windows 11, x86_64  
**Ejecutor:** Antigravity (Gemini 3.8 Flash High)

---

## 1. Resumen Ejecutivo

El repositorio `Kinova-Gen3-Geometric-Simulator` proporciona una implementación mecánica rigurosa y autocontenida (MATLAB base puro, sin toolboxes de pago ni dependencias MEX/Simulink/Python en tiempo de ejecución) del manipulador Kinova Gen3 de 6 grados de libertad (6 DoF), configuración *no-vision* y brazo desnudo (*bare arm*).

La implementación original (**Versión A**) modela la configuración articular en coordenadas euclidianas $q \in \mathbb{R}^6$, sustentándose en el formalismo de Lie $SE(3)$, Product of Exponentials (PoE), transformaciones adjuntas y el algoritmo recursivo de Newton-Euler (RNEA) espacial con optimizaciones de hoisting de invariantes y caché de estados.

La auditoría confirma que:
1. La suite original de pruebas pasa al 100% con errores residuales en el rango de precisión de máquina (`double`).
2. El benchmark local reproduce fielmente el rendimiento histórico: aceleración de **6.75x** del RNEA optimizado frente a la referencia canónica de Modern Robotics (145.40 µs vs. 981.40 µs).
3. La simulación nominal con lazo cerrado ZOH (4 ms / substep 0.5 ms) exhibe un error máximo de balance energético de **3.26 × 10⁻⁹ J**.
4. La geometría del robot define 6 articulaciones revolutas con ejes locales coincidentes con $\hat{z}$ en todos los marcos locales ($A_i = [0;0;1;0;0;0]^T$).
5. Fundamentalmente, a diferencia del Franka Panda (donde las 7 juntas tienen límites acotados), el Kinova Gen3 contiene **tres articulaciones continuas sin topes angulares** (J1, J4, J6) y **tres articulaciones revolutas acotadas** (J2, J3, J5).

---

## 2. Mapa Arquitectural y Dependencias

```
Kinova-Gen3-Geometric-Simulator/
├── +kinova/                                 # Espacio de nombres MATLAB (Versión A)
│   ├── config.m                            # Configuración de simulación, controlador y callbacks
│   ├── draw_robot.m                        # Renderizado 3D de eslabones y mallas STL
│   ├── dynamics.m                          # Matriz M(q) completa y bias b = c(q,dq) + g(q) + f(dq)
│   ├── energy.m                            # Hamiltoniano mecánico H = 1/2 dq'*M*dq + V(q)
│   ├── fk.m                                # Cinemática directa PoE espacial (SE(3))
│   ├── forward_dynamics.m                  # Dinámica directa ddq = M \ (tau + Jb'*F_env - b)
│   ├── frames.m                            # Cinemática directa recursiva de eslabones sucesivos
│   ├── friction.m                          # Fricción suave Muraccioli et al. RSS 2025
│   ├── inverse_dynamics.m                  # Dinámica inversa RNEA: esfuerzo para aceleración deseada
│   ├── jacobian.m                          # Jacobiano de twist espacial y corporal (6x6, [omega; v])
│   ├── joint_vector.m                      # Validación y coerción de vector articular (6x1 double)
│   ├── lagrange_reference.m                # Cálculo independiente COM-Jacobiano/Lagrange (oráculo)
│   ├── model.m                             # Construcción del modelo, DH/URDF, inercias y tornillos
│   ├── plot_results.m                      # Diagnósticos gráficos y figuras PNG
│   ├── read_stl.m                          # Lector binario de mallas STL para visualización
│   ├── reference.m                         # Generador de trayectorias quintic (ftv5)
│   ├── rpy_rotation.m                      # Matriz SO(3) roll-pitch-yaw estándar
│   ├── simulate.m                          # Integrador RK4 (0.5 ms) con lazo ZOH (4 ms)
│   └── write_json.m                        # Serializador de métricas a JSON
├── third_party/
│   ├── modern_robotics/                    # Librería Lynch & Park (MIT) para SE(3)
│   ├── shape_kernels/                      # Kernels RNEA optimizados con hoisting y caché (MIT)
│   │   ├── rnea_shared_kinematics_MR.m     # Precálculo de cinemática compartida y caché bitwise
│   │   ├── InverseDynamicsSharedKinematics_MR.m # RNEA hacia adelante y atrás con ad(V) directo
│   │   └── MassMatrixZeroStateSharedKinematics_MR.m # Matriz de masa P3 sin bias de gravedad
│   └── flatness/
│       └── +ftv5/                          # Planificador de fase quintic diferencialmente plano
├── sources/                                # Metadatos de ingeniería y mallas oficiales
│   ├── parameters.json                     # Parámetros físicos (URDF 5f66bbd, Cholet, Muraccioli)
│   ├── kinova_gen3_6dof_novision.urdf      # URDF oficial de Kinova Robotics
│   ├── manifest.json / mesh_manifest.json  # Hashes de procedencia SHA-256
│   └── meshes/                             # Archivos STL oficiales de eslabones 0 a 6
├── tests/                                  # Banco de pruebas de regresión y validación física
│   ├── validate_kinova.m                   # 40 estados aleatorios contra oráculo de Lagrange
│   └── test_simulation_inputs.m            # Verificación de efectividad, saturación y wrench externo
├── tools/
│   ├── benchmark_kinova.m                  # Benchmark stock vs optimizado (100 estados)
│   ├── prepare_sources.py                  # Extractor de URDF a parameters.json
│   └── verify_sources.py                   # Verificador criptográfico SHA-256
├── run_kinova_demo.m                        # Script maestro de demostración y paso fino
└── setup_kinova.m                           # Inicializador de paths de MATLAB
```

---

## 3. Parámetros Físicos y Cinemáticos del Kinova Gen3

### 3.1. Articulaciones y Límites

Procedencia: URDF oficial `5f66bbd1be84cb699020621dc7c8f3df01394960` y `sources/parameters.json`.

| Articulación | Tipo | Límite Inferior ($q_{\min}$) | Límite Superior ($q_{\max}$) | Esfuerzo Máximo ($\tau_{\max}$) | Velocidad Máxima ($\dot{q}_{\max}$) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **Joint 1** | **Continuous** | $-\infty$ | $+\infty$ | $39.0\text{ N}\cdot\text{m}$ | $1.3963\text{ rad/s}$ |
| **Joint 2** | **Revolute** | $-2.24\text{ rad}$ | $+2.24\text{ rad}$ | $39.0\text{ N}\cdot\text{m}$ | $1.3963\text{ rad/s}$ |
| **Joint 3** | **Revolute** | $-2.57\text{ rad}$ | $+2.57\text{ rad}$ | $39.0\text{ N}\cdot\text{m}$ | $1.3963\text{ rad/s}$ |
| **Joint 4** | **Continuous** | $-\infty$ | $+\infty$ | $9.0\text{ N}\cdot\text{m}$ | $1.2218\text{ rad/s}$ |
| **Joint 5** | **Revolute** | $-2.09\text{ rad}$ | $+2.09\text{ rad}$ | $9.0\text{ N}\cdot\text{m}$ | $1.2218\text{ rad/s}$ |
| **Joint 6** | **Continuous** | $-\infty$ | $+\infty$ | $9.0\text{ N}\cdot\text{m}$ | $1.2218\text{ rad/s}$ |

### 3.2. Ejes de Tornillo Locales

Verificación ejecutada en MATLAB para $i = 1, \dots, 6$:
$$\mathcal{A}_i = \text{Ad}_{M_i^{-1} \cdots M_1^{-1}} \mathcal{S}_i = \begin{bmatrix} 0 \\ 0 \\ 1 \\ 0 \\ 0 \\ 0 \end{bmatrix}$$
Todos los ejes son giros puros en torno al eje $z$ de sus marcos de eslabón respectivos.

### 3.3. Inercia Aparente del Actuador ($J_{\text{app}}$)

Modelo nominal `cholet_apparent` (Cholet 2025, Tablas A.1/B.1):
$$J_{\text{app}} = \text{diag}(0.0930079,\, 0.0930079,\, 0.0930079,\, 0.0560033,\, 0.0560033,\, 0.0560033)\text{ kg}\cdot\text{m}^2$$
Modelos alternativos soportados: `rigid_links` ($J_{\text{app}} = 0$) y `custom` (definido por usuario).
Interviene aditivamente en $M(q) = M_{\text{links}}(q) + J_{\text{app}}$, en la dinámica inversa $\tau = \tau_{\text{rigid}} + J_{\text{app}}\ddot{q} + f(\dot{q})$, y en la energía cinética $K = \frac{1}{2}\dot{q}^T M \dot{q}$.

### 3.4. Ley de Fricción

Modelo nominal `muraccioli_smooth` (Muraccioli et al., RSS 2025):
$$f_i(\dot{q}_i) = \beta_{v,i} \dot{q}_i + \beta_{c,i} \tanh(\gamma \dot{q}_i)$$
con $\gamma = 50$, y perfiles alternativos `shape_blend` ($f_i \cdot \tanh(\gamma |\dot{q}_i|)$) y `none`.
- Juntas 1–3: $\beta_v = 2.0\text{ N}\cdot\text{m}\cdot\text{s/rad}$, $\beta_c = 2.15\text{ N}\cdot\text{m}$.
- Juntas 4–6: $\beta_v = 1.36\text{ N}\cdot\text{m}\cdot\text{s/rad}$, $\beta_c = 1.60\text{ N}\cdot\text{m}$.

---

## 4. Resultados del Baseline Ejecutado (Hito H0)

### 4.1. Suite de Validación Mecánica (`tests/validate_kinova.m`)

Ejecución sobre 40 estados aleatorios con semilla determinista `13102026`:
- `mass_reference_error`: $9.26 \times 10^{-16}$
- `gravity_reference_error_Nm`: $2.28 \times 10^{-14}\text{ N}\cdot\text{m}$
- `coriolis_reference_error_Nm`: $1.75 \times 10^{-10}\text{ N}\cdot\text{m}$ (diferencias finitas $\epsilon = 10^{-5}$)
- `fk_chain_error`: $2.24 \times 10^{-15}$
- `jacobian_finite_difference_error`: $8.32 \times 10^{-11}$
- `mass_asymmetry`: $2.52 \times 10^{-16}$
- `min_mass_eigenvalue`: $0.056180\text{ kg}\cdot\text{m}^2 > 0$ (definida positiva estricta)
- `potential_gradient_error_Nm`: $6.55 \times 10^{-10}\text{ N}\cdot\text{m}$
- `power_identity_error`: $2.13 \times 10^{-10}\text{ W}$ ($\dot{q}^T c = \frac{1}{2}\dot{q}^T \dot{M}\dot{q}$)
- `stock_inverse_dynamics_error_Nm`: $0$
- `stock_mass_error`: $0$
- `forward_inverse_roundtrip_error`: $5.46 \times 10^{-14}\text{ rad/s}^2$
- `wrench_duality_error`: $1.78 \times 10^{-15}\text{ W}$ ($F^T (J_b \dot{q}) = (J_b^T F)^T \dot{q}$)
- `minimum_friction_power_W`: $5.0978\text{ W} \ge 0$ (pasividad estricta)
- `adjoint_exponential_identity_error`: $2.50 \times 10^{-15}$
- **Dictamen:** **PASS** (100% verificado).

### 4.2. Validación de Entradas y Perturbaciones (`tests/test_simulation_inputs.m`)

- `forced_energy_balance_error_J`: $6.85 \times 10^{-9}\text{ J}$
- `external_work_J`: $3.43 \times 10^{-5}\text{ J}$
- `saturation_test_max_utilization`: $1.000000$ (saturación exacta a $\tau_{\max}$)
- `edge_pose_error`: $6.93 \times 10^{-16}$
- **Dictamen:** **PASS**.

### 4.3. Benchmark de Dinámica (`tools/benchmark_kinova.m`)

- Muestreo: 100 estados aleatorios, 5 repeticiones en CPU serie precalentada.
- Modern Robotics canónico (`stock_median_us`): **981.40 µs**
- RNEA espacial optimizado (`optimized_median_us`): **145.40 µs**
- **Aceleración:** **6.75x**

### 4.4. Demostración y Simulación Nominal (`run_kinova_demo.m`)

- Trayectoria quintic de 3.6 s ($q_0 = [0; 0.35; -0.7; 0; 0.35; 0] \to q_{\text{goal}} = [0.35; 0.6; -0.95; 0.3; 0.55; -0.25]$).
- Error de balance de energía ($h = 0.5\text{ ms}$): **$3.26 \times 10^{-9}\text{ J}$**.
- Paso fino ($h = 0.25\text{ ms}$): **$2.73 \times 10^{-9}\text{ J}$**.
- Error máximo de seguimiento en posición: $0.229\text{ mm}$.
- Error máximo articular: $0.190^\circ$.
- Violación de límites de etapa: $0\text{ rad}$.
- **Dictamen:** **PASS**.

---

## 5. Control de Versiones y Rama Experimental

- Tag base reproducible: `v0.1.0` sobre commit `e2aefe2`.
- Rama experimental creada: `experiment/kinova-s1-dynamics`.
- No se detectaron modificaciones no rastreadas ni archivos ajenos al simulador.
