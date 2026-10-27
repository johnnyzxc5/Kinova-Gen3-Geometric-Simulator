# Integración Geométrica, Restricciones y Consistencia Física — Kinova Gen3

**Hito:** HITO H2 — Tests, simulaciones y benchmarks  
**Fecha:** 27 de octubre de 2026  
**Autor / Modelo:** Antigravity (Gemini 3.8 Flash High)  
**Repositorio:** `C:\Users\Johnny\Desktop\Git\Kinova-Gen3-Geometric-Simulator`

---

## 1. Integrador de Lie Munthe-Kaas en $(S^1)^6 \times \mathbb{R}^6$

### 1.1. Ecuaciones Diferenciales sobre la Variedad
El espacio de estados de la formulación global no es el espacio vectorial euclidiano $\mathbb{R}^{12}$, sino el fibrado tangente trivializado:
$$T\mathsf{Q} = (S^1)^6 \times \mathbb{R}^6$$
donde $X = [x_1, \dots, x_6] \in (S^1)^6 \subset \mathbb{R}^{2\times 6}$ y $\omega = (\omega_1, \dots, \omega_6)^T \in \mathbb{R}^6$.

La evolución continua del sistema viene dada por:
$$\begin{aligned}
\dot{x}_i &= \omega_i J x_i, \qquad J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}, \qquad i = 1,\dots,6, \\
\dot{\omega} &= M(X)^{-1} \left( \tau_{\text{applied}} + J_b(X)^T F_{\text{env}} - b(X, \omega) \right).
\end{aligned}$$

### 1.2. Algoritmo Abelian Munthe-Kaas RK4
Dado que el grupo de Lie $(SO(2))^6 \cong (S^1)^6$ es abeliano, el corchete de Lie en el álgebra se anula idénticamente ($[\xi, \eta] = 0$). Como consecuencia:
$$\operatorname{dexp}_{\Delta\theta}^{-1} = I_6$$
El integrador de Munthe-Kaas de cuarto orden (RK4-MK) evalúa las cuatro etapas clásicas directamente en el álgebra de Lie $\mathfrak{so}(2)^6 \cong \mathbb{R}^6$:
$$\begin{aligned}
\text{Etapa 1:} \quad & X_1 = X(t_k), \quad \omega_1 = \omega(t_k), \\
& \Delta\omega_1 = \omega_1, \quad k_{1,\omega} = \operatorname{RHS}_\omega(t_k, X_1, \omega_1), \\[6pt]
\text{Etapa 2:} \quad & X_2 = \operatorname{Exp}_{S^1}\left(\frac{h}{2}\Delta\omega_1, X_1\right), \quad \omega_2 = \omega_1 + \frac{h}{2} k_{1,\omega}, \\
& \Delta\omega_2 = \omega_2, \quad k_{2,\omega} = \operatorname{RHS}_\omega\left(t_k + \frac{h}{2}, X_2, \omega_2\right), \\[6pt]
\text{Etapa 3:} \quad & X_3 = \operatorname{Exp}_{S^1}\left(\frac{h}{2}\Delta\omega_2, X_1\right), \quad \omega_3 = \omega_1 + \frac{h}{2} k_{2,\omega}, \\
& \Delta\omega_3 = \omega_3, \quad k_{3,\omega} = \operatorname{RHS}_\omega\left(t_k + \frac{h}{2}, X_3, \omega_3\right), \\[6pt]
\text{Etapa 4:} \quad & X_4 = \operatorname{Exp}_{S^1}\left(h \Delta\omega_3, X_1\right), \quad \omega_4 = \omega_1 + h k_{3,\omega}, \\
& \Delta\omega_4 = \omega_4, \quad k_{4,\omega} = \operatorname{RHS}_\omega(t_k + h, X_4, \omega_4), \\[6pt]
\text{Actualización:} \quad & \Delta\theta_{\text{step}} = \frac{h}{6} (\Delta\omega_1 + 2\Delta\omega_2 + 2\Delta\omega_3 + \Delta\omega_4), \\
& X(t_k + h) = \operatorname{Exp}_{S^1}(\Delta\theta_{\text{step}}, X_1), \\
& \omega(t_k + h) = \omega_1 + \frac{h}{6} (k_{1,\omega} + 2k_{2,\omega} + 2k_{3,\omega} + k_{4,\omega}).
\end{aligned}$$

### 1.3. Preservación de la Variedad a Nivel de Máquina
La acción de grupo $\operatorname{Exp}_{S^1}(\Delta\theta, X)$ rota exactamente cada columna mediante la matriz ortogonal $R_z(\Delta\theta_i) \in SO(2)$. Con una renormalización radial por columna tras la actualización de etapa:
$$\|x_i\|_2 = 1 \pm 2.22 \times 10^{-16}$$
Esta invariancia se mantiene estrictamente durante simulaciones arbitrariamente largas, sin acumulación de deriva de norma y sin distorsión de fase.

---

## 2. Tratamiento de Juntas Continuas y Conservación de Winding

### 2.1. Topología del Círculo Completo
Las articulaciones 1, 4 y 6 no poseen topes mecánicos ($q \in \mathbb{R}$ o $x \in S^1$).
La condición de admisibilidad en `check_limits` se reduce a:
$$\|x_i\|_2 = 1 \pm \text{tol}_{\text{norm}}$$
con margen $\mu_i = +\infty$. No se imponen desigualdades artificiales de tope.

### 2.2. Contrato de Vueltas Acumuladas (Winding)
Una rotación de $q$ y $q + 2\pi k$ produce la misma orientación física $x = (\cos q, \sin q)^T \in S^1$. Sin embargo, en robótica una trayectoria prescrita que comanda múltiples vueltas (p. ej. enroscar un tornillo o desenrollar un cable con $4\pi$ radianes) tiene una semántica física no equivalente a $0$ radianes.

El simulador resuelve esta distinción mediante un contrato formal:
1. **Durante la simulación continua (`kinova_s1.simulate`):**  
   El integrador acumula el avance angular $q_{\text{unwrapped}}(t) = q_0 + \sum \Delta\theta_{\text{step}}$. De esta forma, la telemetría y los registros `log.q` reflejan el recorrido acumulado continuo en $\mathbb{R}$ sin saltos de discontinuidad de $2\pi$.
2. **En la función de levantamiento (`kinova_s1.to_q(X, m, q_prev)`):**
   - Si se proporciona el estado continuo previo $q_{\text{prev}}$, la función calcula el desplazamiento relativo en $(-\pi, \pi]$ respecto a $q_{\text{prev}}$:
     $$\Delta_i = \operatorname{atan2}(s_i \cos q_{\text{prev},i} - c_i \sin q_{\text{prev},i}, \; c_i \cos q_{\text{prev},i} + s_i \sin q_{\text{prev},i})$$
     $$q_i = q_{\text{prev},i} + \Delta_i$$
     preservando exactamente el número entero de vueltas $k \in \mathbb{Z}$.
   - Si $q_{\text{prev}}$ no se proporciona, la función reconoce formalmente la pérdida de información sobre $k$ y devuelve la rama principal $\operatorname{atan2}(s_i, c_i) \in (-\pi, \pi]$.
3. **En el operador de desplazamiento (`kinova_s1.box_diff`) y en el lazo de control (`kinova_s1.simulate`):**
   - Para las juntas continuas $\{1, 4, 6\}$, `box_diff(X2, X1, m, winding)` calcula la diferencia geodésica mínima $d_{\text{geo},i}(X_2, X_1) \in (-\pi, \pi]$ más el desplazamiento de fibra entera $2\pi w_i$ ($w_i \in \mathbb{Z}$).
   - En el modo predeterminado de seguimiento de trayectorias parametrizadas (`c.continuous_control_mode = 'unwrapped'`), el controlador evalúa el índice entero de fibra entre la referencia levantada $q_{r,i}(t)$ y el estado continuo levantado $\tilde{q}_i(t)$:
     $$w_i = \operatorname{round}\!\left(\frac{(q_{r,i} - \tilde{q}_i) - d_{\text{geo},i}(X_r, X)}{2\pi}\right) \in \mathbb{Z}$$
     y pasa $w_i$ a `kinova_s1.box_diff(Xr, X, m, w)`. Esto garantiza que, incluso si una articulación continua acumula un retraso transitorio $|q_{r,i} - \tilde{q}_i| > \pi$ (por ejemplo, bajo saturación de par $\tau_{\max,6} = 9\,\text{N}\cdot\text{m}$), el controlador nunca sufre pérdida de vuelta (*winding slip*).
   - Si el usuario especifica explícitamente `c.continuous_control_mode = 'modular_s1'`, se fija $w_i \equiv 0$, de modo que el controlador busca el camino circular más corto sobre $S^1$ módulo $2\pi$.
4. **En las etapas RK4-MK de entradas externas (`external_wrench_function`):**
   - Cada etapa intermedia $s \in \{1,2,3,4\}$ propaga su levantamiento continuo $q_{\text{stage},s} = q_{\text{unwrapped}} + \Delta\theta_s$ hacia `kinova_s1.to_q(X_s, m, q_{\text{stage},s})`, garantizando que fuerzas o pares externos dependientes de la posición acumulada $F_{\text{ext}}(t, q, \omega)$ sean suaves al cruzar $\pm\pi$.

---

## 3. Articulaciones Acotadas y Monitoreo de Topes

### 3.1. Caracterización de Arcos en $(S^1)$
Para las articulaciones 2, 3 y 5, los límites mecánicos son:
- J2: $[-2.24, 2.24]\text{ rad}$ ($h_2 = 2.24$)
- J3: $[-2.57, 2.57]\text{ rad}$ ($h_3 = 2.57$)
- J5: $[-2.09, 2.09]\text{ rad}$ ($h_5 = 2.09$)

Debido a que son simétricos, el punto medio es $m_i = 0$, el vector central es $u_i = (1, 0)^T$, y el umbral de arco es $\cos(h_i)$.

La condición de arco admisible es:
$$u_i^T \hat{x}_i \ge \cos h_i - \text{tol}, \qquad \mu_i = h_i - |\operatorname{atan2}(\hat{s}_i, \hat{c}_i)| \ge -\text{tol}$$

### 3.2. Consistencia Homotópica de `box_diff`
El operador `kinova_s1.box_diff` calcula el desplazamiento dentro del arco admisible:
$$dq_{\text{box},i} = \delta_{2,i} - \delta_{1,i} \in [-2h_i, 2h_i]$$
Dado que el corte de rama se ubica en $m_i \pm \pi = \pm\pi$ (en el sector prohibido $S^1 \setminus A_i$), la interpolación a lo largo de $dq_{\text{box}}$ garantiza que la trayectoria de referencia jamás comande movimiento a través del tope mecánico duro, incluso cuando $|\delta_{2,i} - \delta_{1,i}| > \pi$ (como ocurre en excursiones amplias de las juntas 2, 3 y 5, cuyo rango total supera $\pi\text{ rad}$).

### 3.3. Monitoreo Intrínseco en $S^1$ vs. Invarianza Dinámica
El integrador `kinova_s1.simulate` monitoriza tanto el error de norma unitaria $\max |c_{s,i}^2 + s_{s,i}^2 - 1|$ como los márgenes intrínsecos al arco admisible $h_i \pm \delta_i(X_s)$ en las cinco configuraciones de cada subpaso RK4-MK ($X_1, X_2, X_3, X_4, X_{k+1}$). Se documenta explícitamente que esto es una **monitorización cinemática pasiva**: bajo esfuerzos saturados o perturbaciones externas severas, evitar la colisión física contra el tope requiere una ley de control activa (p. ej. Control Barrier Functions, CBF) o un modelo de contacto con restitución, los cuales son independientes del integrador geométrico de la planta.

---

## 4. Consistencia Energética, Convergencia $O(h^4)$ y Efecto `NearZero`

### 4.1. Balance de Potencia y Hamiltoniano Mecánico
El Hamiltoniano mecánico total es:
$$H(X, \omega) = \frac{1}{2}\omega^T M(X) \omega + V(X)$$
donde $M(X) = M_{\text{links}}(X) + J_{\text{app}}$ incluye la inercia aparente exactamente una vez.

La derivada temporal a lo largo de las trayectorias del sistema satisface:
$$\dot{H} = \omega^T \tau_{\text{applied}} - \omega^T f(\omega) + \omega^T (J_b(X)^T F_{\text{env}})$$
Integrando en el tiempo:
$$H(t) - H(0) = W_{\text{joint}}(t) - E_{\text{friction}}(t) + W_{\text{env}}(t)$$
El residuo de balance de energía se define como:
$$\Delta E(t) = H(t) - H(0) - W_{\text{joint}}(t) - W_{\text{env}}(t) + E_{\text{friction}}(t)$$

### 4.2. Diagnóstico Exacto del Estancamiento `NearZero` en la Versión A frente a la Convergencia $O(h^4)$ de la Versión B
En la Versión A (`+kinova/`), el kernel `rnea_shared_kinematics_MR.m` (líneas 91–92) invoca `MatrixExp6(VecTose3(cached_Ai(:,i) * -thetalist(i)))`. Dentro de `MatrixExp6`, cuando $\|\omega\theta\| < 10^{-6}$ (`NearZero`), la rotación se trunca abruptamente a la matriz identidad $I_3$, introduciendo una discontinuidad de orden $O(10^{-6})$ en el campo vectorial dinámico.

En la trayectoria nominal del Kinova Gen3, la condición inicial $q_0 = [0, 0.35, -0.7, 0, 0.35, 0]^T$ parte con $q_1(0) = q_4(0) = q_6(0) = 0$. Al arrancar el movimiento, las articulaciones 1, 4 y 6 atraviesan el umbral $|q_i| = 10^{-6}\text{ rad}$ en $t = 0.0360\text{ s}$ ($k = 10$, donde $|q_1| = 3.505 \times 10^{-6}$, $|q_4| = 8.476 \times 10^{-7}$, $|q_6| = 7.662 \times 10^{-7}$), provocando un salto espurio e irreducible de **$2.7271 \times 10^{-9}\text{ J}$** en el residuo energético de la Versión A.

- A paso grueso ($h = 0.5\,\text{ms}$), el pico global de truncamiento RK4 durante la fase de máxima aceleración/velocidad ($3.2580 \times 10^{-9}\text{ J}$) supera ligeramente el salto de `NearZero` ($2.7271 \times 10^{-9}\text{ J}$), por lo que ambas versiones muestran errores máximos casi idénticos ($3.258 \times 10^{-9}\text{ J}$).
- Sin embargo, al refinar el paso de integración ($h = 0.5\,\text{ms} \to 0.25\,\text{ms} \to 0.125\,\text{ms}$), **la Versión A se estanca por completo** en el piso de discontinuidad de `NearZero` ($2.7271 \times 10^{-9}\text{ J}$), mientras que **la Versión B en $(S^1)^6$** (cuya cinemática algebraica es analítica $C^\infty$ y libre de `NearZero`) mantiene una convergencia estricta de cuarto orden $O(h^4)$:

| Paso de planta $h$ | Versión A ($\mathbb{R}^6$, `MatrixExp6`) $\max|\Delta E|$ | Ratio A | Versión B ($(S^1)^6$, algebraica) $\max|\Delta E|$ | Ratio B | Ventaja de precisión B vs. A |
|---|---|---|---|---|---|
| $0.500\,\text{ms}$ (`substeps = 8`) | $3.2580 \times 10^{-9}\text{ J}$ | — | $3.2578 \times 10^{-9}\text{ J}$ | — | $1.00\times$ |
| $0.250\,\text{ms}$ (`substeps = 16`) | $2.7271 \times 10^{-9}\text{ J}$ | $1.19\times$ (estancado) | $1.6663 \times 10^{-10}\text{ J}$ | **$19.55\times$** | **$16.37\times$** |
| $0.125\,\text{ms}$ (`substeps = 32`) | $2.7271 \times 10^{-9}\text{ J}$ | $1.00\times$ (estancado) | $9.3765 \times 10^{-12}\text{ J}$ | **$17.77\times$** | **$290.8\times$** |

Este resultado demuestra rigurosamente por qué la formulación geométrica global en $(S^1)^6$ no sólo acelera la evaluación puntual de RNEA, sino que elimina la barrera de no-suavidad de `MatrixExp6`/`NearZero` y restaura la convergencia asintótica $O(h^4)$ del integrador de Munthe-Kaas.
