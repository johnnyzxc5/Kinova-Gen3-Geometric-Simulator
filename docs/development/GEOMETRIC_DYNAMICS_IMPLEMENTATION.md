# Implementación de Dinámica Geométrica en $(S^1)^6$ — Kinova Gen3

**Hito:** HITO H1 — Adaptación e Implementación $S^1$  
**Fecha:** 27 de octubre de 2026  
**Autor / Modelo:** Antigravity (Gemini 3.8 Flash High)  
**Repositorio:** `C:\Users\Johnny\Desktop\Git\Kinova-Gen3-Geometric-Simulator`

---

## 1. Resumen de la Arquitectura Geométrica Implementada

En el Hito H1 se ha desarrollado e integrado el núcleo cinemático y dinámico global en $(S^1)^6$ (`+kinova_s1/` y `third_party/shape_kernels/rnea_shared_kinematics_S1.m`), adaptado rigurosamente al manipulador Kinova Gen3 de 6 grados de libertad.

Cada operador evalúa **intrínseca y algebraicamente sobre $(S^1)^6$**, eliminando conversiones intermedias de ángulo (`to_q` / `atan2`) y llamadas trascendentales (`cos`, `sin`, `MatrixExp6`) en las 6 articulaciones nominales, preservando a la vez compatibilidad exacta de fallback ante perturbaciones helicoidales de tornillos.

---

## 2. Tratamiento Riguroso de la Topología: Juntas Continuas vs. Acotadas

A diferencia de robots como el Franka Panda (donde todas las articulaciones están confinadas a sectores acotados), el Kinova Gen3 presenta una topología mixta:
$$Q \cong (S^1)^6, \qquad \mathcal{D}_{\text{adm}} = S^1 \times A_2 \times A_3 \times S^1 \times A_5 \times S^1 \subset (S^1)^6$$

### 2.1. Articulaciones Bounded (2, 3, 5)
- Arcos físicos acotados: $q_2 \in [-2.24, 2.24]$, $q_3 \in [-2.57, 2.57]$, $q_5 \in [-2.09, 2.09]\text{ rad}$.
- Punto medio simétrico: $m_i = 0$, semiamplitud $h_i = q_{\max,i} < \pi$.
- Vector unitario central: $u_i = (\cos m_i, \sin m_i) = (1, 0)$.
- Condición de admisibilidad en `check_limits`:
  $$u_i^T \hat{x}_i \ge \cos(h_i) - \text{tol}, \qquad \mu_i = h_i - |\delta_i| \ge -\text{tol}$$
  donde $\hat{x}_i = x_i / \|x_i\|$ y $\delta_i = \operatorname{atan2}(s_i, c_i) \in (-h_i, h_i)$.
- Desplazamiento `box_diff`: evalúa $\delta_{2,i} - \delta_{1,i} \in [-2h_i, 2h_i]$, garantizando que la guía cinemática jamás mande trayectorias a través del tope prohibido $S^1 \setminus A_i$.

### 2.2. Articulaciones Continuas (1, 4, 6)
- Círculos completos $S^1$ sin topes mecánicos en el URDF.
- Condición en `check_limits`: pertenencia estricta a la variedad $|c_i^2 + s_i^2 - 1| \le \text{tol}_{\text{norm}}$, con margen $\mu_i = +\infty$.
- **Contrato de levantamiento y vueltas acumuladas (Winding):**
  - En $(S^1)^6$, $x_i = (\cos q_i, \sin q_i)$ representa la orientación física exacta sin singularidades.
  - La función `kinova_s1.to_q(X, m, q_prev)` implementa un contrato claro:
    1. Si se proporciona $q_{\text{prev}}$ (p. ej. en la integración temporal o en lazos de control con referencia extendida), calcula el desplazamiento relativo en $(-\pi, \pi]$ respecto a $q_{\text{prev}}$ y acumula $q(t)$, preservando el número de vueltas $k \in \mathbb{Z}$.
    2. Si no se suministra $q_{\text{prev}}$, devuelve la rama principal $\operatorname{atan2}(s_i, c_i) \in (-\pi, \pi]$, reconociendo formalmente que $X$ por sí sola no almacena el índice de homotopía $k$.
  - En `box_diff`: para juntas continuas devuelve la diferencia geodésica mínima en $(-\pi, \pi]$, salvo que se especifique un vector de vueltas enteras `winding`.

---

## 3. Homomorfismo del Adjunto en `rnea_shared_kinematics_S1.m`

Para las 6 articulaciones del Kinova Gen3, los tornillos locales son puros en $\hat{z}$:
$$\mathcal{A}_i = [0, 0, 1, 0, 0, 0]^T, \qquad \forall i \in \{1,\dots,6\}$$
Por tanto, $\exp(-[\mathcal{A}_i]q_i) = \operatorname{diag}(R_z(x_i)^T, 1)$ tiene traslación idénticamente nula.
Por el homomorfismo del adjunto:
$$\operatorname{Ad}_{T_{i,i-1}(x_i)} = \begin{bmatrix} R_z(x_i)^T & 0_{3\times 3} \\ 0_{3\times 3} & R_z(x_i)^T \end{bmatrix} B^{(i)}, \qquad B^{(i)} := \operatorname{Ad}_{M_i^{-1}} \in \mathbb{R}^{6\times 6}$$
donde $R_z(x_i)^T = \begin{bmatrix} c_i & s_i & 0 \\ -s_i & c_i & 0 \\ 0 & 0 & 1 \end{bmatrix}$.

En tiempo de ejecución, calcular $\operatorname{Ad}_{T_{i,i-1}}$ para las 6 articulaciones requiere únicamente rotaciones planas bilineales de las filas de $B^{(i)}$, reduciendo drásticamente las operaciones matriciales y eliminando `MatrixExp6`.

---

## 4. Preservación de Parámetros Físicos Originales

- **Inercia Aparente ($J_{\text{app}}$):** Se conservan los perfiles `cholet_apparent`, `rigid_links` y `custom`. En `kinova_s1.dynamics`, `inverse_dynamics`, `forward_dynamics` y `energy`, $J_{\text{app}}$ se añade exactamente una sola vez.
- **Fricción:** Ley suave de Muraccioli et al. (RSS 2025) con parámetros idénticos a los del modelo original.
- **Entradas externas:** Se respetan rigurosamente las convenciones de signos de wrenches externos aplicados en la brida (`external_wrench`) y su correspondencia con $F_{\text{tip}} = -\text{external\_wrench}$ y $J_b(X)^T F$.
