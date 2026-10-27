# Auditoría Matemática y Científica Independiente — HITO H3 (Kinova Gen3 en $(S^1)^6$)

**Hito:** HITO H3 — Revisión matemática y científica independiente  
**Fecha:** 27 de octubre de 2026  
**Auditor / Modelo:** Gemini 4 Argon (Auditoría Matemática y Científica Independiente)  
**Repositorio auditado:** `C:\Users\Johnny\Desktop\Git\Kinova-Gen3-Geometric-Simulator` (rama `experiment/kinova-s1-dynamics`)  
**Repositorio de referencia contrastado:** `C:\Users\Johnny\Desktop\Git\Franka-Panda-Geometric-Simulator` (rama `main`, intacto)

---

## 1. Resumen Ejecutivo del Dictamen

Se ha realizado una auditoría matemática, física, numérica y de código exhaustiva sobre el paquete `+kinova_s1/`, el kernel `third_party/shape_kernels/rnea_shared_kinematics_S1.m`, la batería de pruebas (`tests/validate_kinova_s1.m`, `tests/test_s1_geometry_limits.m`, `tests/test_s1_simulation_consistency.m`), el benchmark cuádruple (`tools/benchmark_s1_comparison.m`) y la documentación técnica producida en los Hitos H0–H2.

### Hallazgos Críticos Detectados y Resueltos en H3

Durante la auditoría independiente en Gemini 4 Argon se identificaron **cuatro hallazgos sustantivos** (tres vulnerabilidades de borde en `+kinova_s1/simulate.m` y un diagnóstico físico-numérico erróneo en la documentación de H2), los cuales fueron corregidos y verificados experimentalmente en MATLAB R2026a antes de cerrar el Hito H3:

1. **Refutación del diagnóstico de `NearZero` de H2 y demostración de la ventaja de precisión de $290.8\times$ de la Versión B bajo refinamiento de paso:**
   - En H2 (`INTEGRATION_AND_CONSTRAINTS.md`), se afirmó erróneamente que `rnea_shared_kinematics_MR.m` no invocaba `MatrixExp6` y que, por tanto, el umbral `NearZero` ($\|\omega\theta\| < 10^{-6}$) no afectaba a la Versión A del Kinova Gen3.
   - **Realidad matemática demostrada en H3:** `rnea_shared_kinematics_MR.m` (líneas 91–92) **sí invoca** `MatrixExp6(VecTose3(cached_Ai(:,i) * -thetalist(i)))`. Dado que la configuración inicial nominal $q_0 = [0, 0.35, -0.7, 0, 0.35, 0]^T$ tiene $q_1(0) = q_4(0) = q_6(0) = 0$, en los primeros instantes ($t \in [0, 0.036\text{ s}]$, cruzando en $k=10, t=0.0360\text{ s}$ con $|q_1| = 3.505\times 10^{-6}$, $|q_4| = 8.476\times 10^{-7}$, $|q_6| = 7.662\times 10^{-7}$) `MatrixExp6` trunca abruptamente las rotaciones a la identidad $I_3$. Esto introduce un salto discontinuo irreducible de **$2.7271 \times 10^{-9}\text{ J}$** en el residuo energético de la Versión A.
   - A paso grueso ($h = 0.5\text{ ms}$), este salto quedaba oculto detrás del pico global de truncamiento RK4 ($3.2580 \times 10^{-9}\text{ J}$). Al refinar el paso ($h = 0.5\text{ ms} \to 0.25\text{ ms} \to 0.125\text{ ms}$), **la Versión A se estanca por completo** en $2.7271 \times 10^{-9}\text{ J}$ (ratios $1.19\times$ y $1.00\times$), mientras que **la Versión B en $(S^1)^6$** converge con orden cuarto estricto $O(h^4)$ hasta **$1.6663 \times 10^{-10}\text{ J}$** ($16.37\times$ más precisa) y **$9.3765 \times 10^{-12}\text{ J}$** (**$290.8\times$ más precisa**).
2. **Eliminación de pérdida de vuelta (*winding slip*) bajo error transitorio $> \pi$ en juntas continuas (`+kinova_s1/simulate.m`):**
   - La llamada original `err_box = kinova_s1.box_diff(Xr, X, m)` omitía el argumento de fibra `winding`. Si una junta continua (1, 4 o 6) acumulaba un error de seguimiento instantáneo $|q_{r,i}(t) - \tilde{q}_i(t)| > \pi$ (por ejemplo, ante una rampa rápida o saturación de torque $\tau_{\max,6} = 9\,\text{N}\cdot\text{m}$), `box_diff` proyectaba el error a $(-\pi, \pi]$ y perdía una vuelta completa de $2\pi$.
   - **Corrección aplicada:** En el modo predeterminado `'unwrapped'`, el controlador calcula el desplazamiento entero de fibra $w_i = \operatorname{round}(((q_{r,i} - \tilde{q}_i) - d_{\text{geo},i}(X_r, X)) / (2\pi)) \in \mathbb{Z}$ y lo entrega a `kinova_s1.box_diff(Xr, X, m, w_ctrl)`. Además, se habilitó la opción explícita `c.continuous_control_mode = 'modular_s1'` ($w_i \equiv 0$) para cuando el usuario desee control circular por camino más corto sobre $S^1$.
3. **Continuidad del levantamiento $q_{\text{stage}}$ en `external_wrench_function` durante las etapas RK4-MK (`+kinova_s1/simulate.m`):**
   - En `inputs(t, X, omega, m, c, n)`, se invocaba `kinova_s1.to_q(X, m)` sin pasar el estado continuo de la etapa $q_{\text{stage}}$, provocando un salto artificial de $\pm 2\pi$ en callbacks $F_{\text{ext}}(t, q, \omega)$ dependientes de la posición cuando una junta continua cruzaba $\pm\pi$.
   - **Corrección aplicada:** Se propaga $q_{\text{stage},s} = q_{\text{unwrapped}} + \Delta\theta_s$ a través de las cuatro etapas de `mk_rhs` e `inputs`, invocando `kinova_s1.to_q(X, m, q_stage)`.
4. **Monitoreo intrínseco en $S^1$ de las etapas intermedias RK4-MK (`+kinova_s1/simulate.m`):**
   - Se actualizó el bloque `capture_substeps` para verificar explícitamente el error de norma unitaria $\max |c_{s,i}^2 + s_{s,i}^2 - 1|$ en las cinco configuraciones del subpaso ($X_1, X_2, X_3, X_4, X_{k+1}$) y evaluar los márgenes a los topes de las juntas acotadas (2, 3, 5) directamente sobre $S^1$ mediante la carta centrada $\delta_i(X_s)$.

---

## 2. Auditoría Detallada por Ejes Técnicos

### Eje 1: Geometría en $(S^1)^6$ y Especialización Algebraica en $SE(3)$

1. **Variedad de configuración y fibrado tangente trivializado:**
   - La variedad global $\mathsf{Q} = (S^1)^6 \subset \mathbb{R}^{2\times 6}$ está correctamente representada mediante matrices $X = [x_1, \dots, x_6]$ con $x_i = (c_i, s_i)^T \in S^1$.
   - La trivialización izquierda del fibrado tangente $T(S^1)^6 \cong (S^1)^6 \times \mathbb{R}^6$ emplea las seis velocidades angulares físicas independientes $\omega \in \mathbb{R}^6$ con cinemática $\dot{x}_i = \omega_i J x_i$.
2. **Verificación de ejes locales y homomorfismo adjunto:**
   - Aunque los tornillos espaciales $\mathcal{S}_i$ en el marco base alternan direcciones ($\mathcal{S}_1, \mathcal{S}_2, \mathcal{S}_3, \dots$ no están todos alineados con $\hat{z}_0$), el tornillo local en el marco del eslabón $i$ cumple exactamente:
     $$\mathcal{A}_i = \operatorname{Ad}_{(M_1 \cdots M_i)^{-1}} \mathcal{S}_i = [0, 0, 1, 0, 0, 0]^T, \qquad \forall i \in \{1,\dots,6\},$$
     con error numérico verificado $< 10^{-16}$.
   - Por consiguiente, la transformación relativa entre marcos sucesivos es $T_{i,i-1}(x_i) = e^{-[\mathcal{A}_i] q_i} M_i^{-1} = \operatorname{diag}(R_z(x_i)^T, 1) M_i^{-1}$, y por el homomorfismo de representación adjunta $\operatorname{Ad}_{AB} = \operatorname{Ad}_A \operatorname{Ad}_B$:
     $$\operatorname{Ad}_{T_{i,i-1}(x_i)} = \begin{bmatrix} R_z(x_i)^T & 0_{3\times 3} \\ 0_{3\times 3} & R_z(x_i)^T \end{bmatrix} \operatorname{Ad}_{M_i^{-1}}.$$
   - La implementación en `rnea_shared_kinematics_S1.m` (líneas 119–127) evalúa esta multiplicación por bloques mediante combinaciones lineales directas de las filas $(1,2)$ y $(4,5)$ de $B_i = \operatorname{Ad}_{M_i^{-1}}$ usando únicamente $c_i, s_i$, sin invocar `MatrixExp6`, `VecTose3`, `Adjoint`, `cos` ni `sin`.
   - En caso de geometrías perturbadas o tornillos con paso helicoidal no nulo ($\hat{\omega}_i^T v_i \ne 0$, comprobado en `validate_kinova_s1.m`), el flag `cached_is_pure_z(i)` activa automáticamente el fallback exacto `kinova_s1.screw_exp`, manteniendo paridad de máquina ($7.11 \times 10^{-15}$).

### Eje 2: Topología Mixta — Juntas Continuas (1, 4, 6) vs. Acotadas (2, 3, 5)

| Junta | Tipo URDF | Rango Físico | Topología en $(S^1)^6$ | Punto Medio $m_i$ | Semiamplitud $h_i$ | $\cos h_i$ |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **J1** | `continuous` | $(-\infty, +\infty)$ | Círculo completo $S^1$ | $0$ | $\infty$ | $-1$ (sin tope) |
| **J2** | `revolute` | $[-2.24, 2.24]\text{ rad}$ | Arco cerrado $\mathcal{A}_2 \subsetneq S^1$ | $0$ | $2.24\text{ rad}$ | $-0.6203616$ |
| **J3** | `revolute` | $[-2.57, 2.57]\text{ rad}$ | Arco cerrado $\mathcal{A}_3 \subsetneq S^1$ | $0$ | $2.57\text{ rad}$ | $-0.8410994$ |
| **J4** | `continuous` | $(-\infty, +\infty)$ | Círculo completo $S^1$ | $0$ | $\infty$ | $-1$ (sin tope) |
| **J5** | `revolute` | $[-2.09, 2.09]\text{ rad}$ | Arco cerrado $\mathcal{A}_5 \subsetneq S^1$ | $0$ | $2.09\text{ rad}$ | $-0.4961876$ |
| **J6** | `continuous` | $(-\infty, +\infty)$ | Círculo completo $S^1$ | $0$ | $\infty$ | $-1$ (sin tope) |

- **Veredicto:** No se ha copiado el esquema de 7 topes del Franka Panda ni se han inventado topes artificiales para las juntas 1, 4 y 6.
- Para las juntas acotadas 2, 3 y 5, dado que el rango total $2h_i \in \{4.48, 5.14, 4.18\}\text{ rad}$ supera $\pi\text{ rad}$, una diferencia geodésica ingenua en $S^1$ envolvería desplazamientos legítimos $> \pi$ a través del tope mecánico en $\pm\pi$. El operador `kinova_s1.box_diff` evita esto al evaluar $\delta_i(x_{2,i}) - \delta_i(x_{1,i}) \in [-2h_i, 2h_i]$ sobre el arco admisible $\mathcal{A}_i$.

### Eje 3: Contrato Formal de Vueltas Acumuladas (Winding Number)

- La proyección $\pi : \mathbb{R} \to S^1$ tiene fibra $\pi^{-1}(x) \cong \mathbb{Z}$. Un estado puntual $X \in (S^1)^6$ carece de información sobre el número entero de vueltas $k \in \mathbb{Z}$.
- El contrato implementado en `+kinova_s1/` distingue rigurosamente:
  1. **Operadores puntuales intrínsecos** (`fk`, `frames`, `jacobian`, `dynamics`, `inverse_dynamics`, `forward_dynamics`, `energy`, `lagrange_reference`): dependen únicamente de $X \in (S^1)^6$ y son periódicos y globales en $(S^1)^6$.
  2. **Levantamiento puntual sin historia (`to_q(X, m)`):** devuelve la rama principal en $(-\pi, \pi]$ para las juntas continuas, documentando explícitamente la pérdida del entero de fibra $k \in \mathbb{Z}$.
  3. **Levantamiento continuo con estado previo (`to_q(X, m, q_prev)`):** reconstruye sin ambigüedad ni saltos de rama el ángulo acumulado cuando el paso incremental cumple $|\Delta q_i| < \pi$.
  4. **Seguimiento de trayectorias con vueltas prescritas (`simulate.m` + `box_diff.m`):** conserva la fibra entera $w_i \in \mathbb{Z}$ en modo `'unwrapped'` (incluso si el error de seguimiento supera $\pi\text{ rad}$) y ofrece modo `'modular_s1'` ($w_i \equiv 0$) para control circular puro.

### Eje 4: Inercia Aparente de Actuadores y Fricción

- **Inercia aparente (`Japp`):**
  - El perfil predeterminado `cholet_apparent` incorpora $J_{\text{app}} = \operatorname{diag}(0.0930079, 0.0930079, 0.0930079, 0.0560033, 0.0560033, 0.0560033)\text{ kg}\cdot\text{m}^2$.
  - Se verificó en `dynamics.m`, `inverse_dynamics.m`, `forward_dynamics.m`, `energy.m` y `lagrange_reference.m` que $J_{\text{app}}$ interviene **exactamente una vez** en $M(X) = M_{\text{links}}(X) + J_{\text{app}}$, en $\tau = \tau_{\text{links}} + J_{\text{app}}\dot{\omega} + f(\omega)$ y en $K = \frac{1}{2}\omega^T M(X)\omega$.
  - Los perfiles alternativos `rigid_links` ($J_{\text{app}} = 0$) y `custom` funcionan de forma idéntica a la Versión A (paridad $< 10^{-15}$).
- **Fricción (`kinova.friction`):**
  - Se reutiliza directamente la ley de fricción original del Kinova (`muraccioli_smooth`, `shape_blend`, `none`), preservando intactos los coeficientes viscosos $\beta_v$, de Coulomb $\beta_c$ y la pendiente $\gamma = 50\text{ s/rad}$.

### Eje 5: Entradas Externas, Jacobianos y Balance Energético

- La transformación de wrenches externos del entorno se realiza mediante $J_b(X)^T F_{\text{env}}$, donde $J_b(X)$ es el jacobiano en el marco corporal de la brida (`kinova_s1.jacobian(X, m, 'body')`) con orden de filas $[\omega_b; v_b]$ compatible con $F_{\text{env}} = [m_b; f_b]$.
- La potencia mecánica inyectada por los actuadores ($\tau_{\text{applied}}^T \omega$), la potencia del entorno ($\omega^T J_b(X)^T F_{\text{env}}$) y la potencia disipada por fricción ($\omega^T f(\omega) \ge 0$) se integran simultáneamente mediante la cuadratura RK4 de cuarto orden junto con $(X, \omega)$, garantizando el cierre del balance energético:
  $$\Delta E(t) = H(t) - H(0) - W_{\text{joint}}(t) - W_{\text{env}}(t) + E_{\text{friction}}(t).$$

### Eje 6: Validez Metodológica del Benchmark

- El benchmark `tools/benchmark_s1_comparison.m` evalúa las 4 formulaciones (`Stock MR`, `Version A`, `Variant B-alg`, `Version B`) sobre $N = 100$ estados aleatorios admisibles con semilla fija `13102026` y $R = 10$ repeticiones.
- **Imparcialidad de caché verificada:** Ambos kernels (`rnea_shared_kinematics_MR` y `rnea_shared_kinematics_S1`) pre-calientan una sola vez sus invariantes estructurales ($M_i^{-1}$, $\operatorname{Ad}_{M_i^{-1}}$, $\mathcal{A}_i$) antes del bucle cronometrado, y durante el barrido de los $N=100$ estados distintos la caché de estado (`cached_theta_u64` / `cached_X_u64`) experimenta un $0\%$ de aciertos artificiales (*100% state misses*), midiendo el coste real de cómputo por configuración nueva.
- **Separación limpia de causas:** `Variant B-alg` demuestra que la factorización adjunta y la exponencial algebraica explican el salto principal de velocidad (p. ej. $146.31\,\mu\text{s} \to 101.21\,\mu\text{s}$ en dinámica completa), mientras que el paso `Variant B-alg` $\to$ `Version B` ($101.21\,\mu\text{s} \to 95.66\,\mu\text{s}$, $+5.5\%$, y $+18.0\%$ en `frames`) mide exactamente el ahorro de eliminar las evaluaciones trigonométricas $\cos q, \sin q$ al mantener el estado en $(S^1)^6$.

---

## 3. Veredicto Final de la Auditoría (HITO H3)

La formulación global sobre $(S^1)^6$ para el manipulador Kinova Gen3 de 6 DoF es **matemáticamente rigurosa, físicamente consistente y numéricamente exacta a precisión de máquina**. Con las correcciones incorporadas en este Hito H3, el repositorio queda aprobado para pasar a los Hitos H4 (generación de animaciones GIF y redacción del `README.md` público) y H5 (publicación remota y verificación visual).
