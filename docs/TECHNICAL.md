# Technical notes

The math behind each part of the engine, with the matching code. Equation numbers refer to
*Unified Particle Physics for Real-Time Applications* (Macklin et al. 2014). Chapter numbers refer to the course
slides.

---

## 1. Rotations (chapter 1)

A rotation of angle θ around the unit axis v is the unit quaternion

```
q = (cos θ/2, v sin θ/2)
```

Going back from a quaternion: `θ = 2 acos(a)` and `v = (b, c, d) / sqrt(1 − a²)`.

A point P rotates with `P' = q (0, P) q⁻¹`, where `q⁻¹ = (a, −b, −c, −d)` for a unit quaternion.

`Quat.Rotate` uses the expanded form `v' = v + 2w (u × v) + 2 u × (u × v)` (u = vector part). It is equal to the
Hamilton product but cheaper; `Quat.RotateHamilton` keeps the literal form, and a test checks that both agree.

Integration of an orientation with angular velocity ω:

- `dq/dt = ½ (0, ω) q`, used inside the ODE solvers (`Quat.Derivative`).
- `q(t+Δt) = exp(ω Δt / 2) q`, exact when ω is constant over the step (`Quat.IntegrateExact`). Used by the
  rigid body world.

---

## 2. Numerical integration (chapter 2 part 1)

A second-order equation `ẍ = F/m` becomes the first-order system `ẋ = v`, `v̇ = F/m`. The state of N particles
is a 6N vector, and the ODE solver only sees `dY/dt = f(Y, t)`.

| Method | Update | Order | Notes |
|---|---|---|---|
| Explicit Euler | `Y += h f(Y, t)` | 1 | Gains energy on oscillators (it explodes in `03_MassSpring`). |
| Semi-implicit Euler | `v += h a(x)`, then `x += h v` | 1 | Symplectic: the energy stays bounded. Used with contacts. |
| Verlet | `x' = 2x − x_prev + h² a` | 2 | Velocity from finite differences. |
| RK4 | `Y += h/6 (k1 + 2k2 + 2k3 + k4)` | 4 | Each `k_i` re-evaluates **all forces** at the intermediate state. |

A version of "RK4" that keeps the acceleration constant during the step is only first order. `OdeSolver` calls
`Derivative` four times and recomputes every force object each time.

---

## 3. Rigid bodies (chapter 2 part 2)

**State:** `X = (x, q, P, L)`.

**Derived quantities:**

```
v = P / M
I⁻¹(t) = R I_body⁻¹ Rᵀ
ω = I⁻¹(t) L
```

**Derivative:**

```
dX/dt = (v, ½ ω q, F, τ)    with    τ = Σ (r_i − x) × F_i
```

`I_body` comes from the shape and the mass (`InertiaTensor`). Any particle cloud works, using the
point-mass sum of slide 31.

Because ω is recomputed from L each step, a free body precesses correctly. A body spinning near its
intermediate axis flips periodically (Dzhanibekov effect, scene `01_RigidBodies`), while L and the kinetic
energy stay constant (checked with RK4).

### Contacts

The normal n points from A to B, and `r_A = p − x_A`. The relative normal velocity is
`v_rel = n · (v_B + ω_B × r_B − v_A − ω_A × r_A)`.

Baraff's impulse that makes `v_rel⁺ = −e v_rel⁻` is

```
j = −(1 + e) v_rel⁻ / (1/m_A + 1/m_B + n · ((I_A⁻¹ (r_A × n)) × r_A) + n · ((I_B⁻¹ (r_B × n)) × r_B))
```

The same denominator, the effective mass, is used for the two friction tangents.

`RigidBodyWorld` solves every contact several times per step. Each pass accumulates the normal impulse,
clamped to `≥ 0`, and the friction impulse, clamped to `[−μ j_n, μ j_n]`.

The target normal velocity is `max(restitution bounce, β/Δt · (penetration − slop))`. This is Baumgarte
stabilization, which removes penetration without explicit position correction.

### Box-box contacts

1. SAT on the 15 axes (3 + 3 face normals, 9 edge cross products) gives the axis of minimum overlap, which is
   the normal and the depth. Nearly parallel edge pairs are skipped.
2. For a face axis, the most anti-parallel face of the other box is clipped (Sutherland-Hodgman) against the 4
   side planes of the reference face. Every clipped point below the reference face becomes a contact, giving
   up to 8 points.
3. For an edge axis, the contact is the midpoint of the closest points of the two edges (segment-segment
   distance).

---

## 4. Collision detection pipeline (chapter 4)

```
integrate → update AABBs → broad phase → narrow phase → contacts (p, n, δ) → response
```

| Broad phase | Cost | Implementation detail |
|---|---|---|
| Brute force | n(n−1)/2 | Reference. |
| Sweep and prune | O(n log n + k), close to O(n) with coherence | The order is kept between frames and re-sorted with an insertion sort, on the axis of largest variance. |
| Uniform grid | ≈ O(n) | A box is inserted in every cell it touches. A pair is reported only in the cell containing the min corner of the two boxes' intersection, so it is never reported twice. |

A test checks that all three return exactly the same pairs on random boxes.

Fast objects tunnel through thin walls when a step's displacement exceeds `e + 2r`.
`TimeOfImpact.SweptSpherePlane` gives the exact impact time for straight motion, and `SubSteps` computes
`n = ⌈vΔt / (e + 2r)⌉` (application 2).

---

## 5. Position based dynamics core (paper §4)

For each constraint `C(x + Δx) = 0` (or `≥ 0`), linearizing and restricting Δx to the mass-weighted gradient
gives (eq. 3-5)

```
Δx = M⁻¹ ∇Cᵀ λ        λ = −C / (∇C M⁻¹ ∇Cᵀ)
```

and, after the iterations, `v = Δx/Δt` (eq. 6).

**Jacobi with averaging (eq. 12-13).** All constraints of a group are evaluated from the same positions.
Their corrections are summed per particle, then divided by the number of constraints n_i acting on the
particle and multiplied by the over-relaxation ω:

```
x*_i += ω / n_i Σ_j Δx_ij
```

This is what lets many constraints act on the same particle without oscillating. In the code, `deltas[i]`
and `deltaCounts[i]` accumulate, and `ApplyDeltas` divides.

**Groups.** Constraint types are solved one group after the other, applying the corrections between groups:

1. fluid density
2. shape contacts
3. particle contacts
4. distance constraints
5. one group per solver extension (the constraints a simulation adds)
6. shape matching

**Extensions.** A simulation adds its own physics without editing the solver by deriving from
`SolverExtension`. The solver calls its hooks at fixed places of Algorithm 1: `ApplyForces` before the
prediction (lines 1-3), `SolveConstraints` once per iteration after the built-in groups (lines 16-22),
`UpdateVelocities` after the velocity update and the fluid forces (lines 24-26), and `EndSubStep` before
`x = x*`. Corrections written with `AddCorrection` are averaged and over-relaxed like every other group.

**Pre-stabilization (§4.4).** Contacts are first solved on the start-of-step positions x, and the correction is
applied to both x and x*. This removes the initial penetration without adding velocity.

**Sleeping (eq. 14).** A particle whose displacement during the step is below ε stays where it was.

---

## 6. Rigid bodies from particles (paper §5)

Shape matching (eq. 15-16):

```
c = Σ m_i x*_i / Σ m_i
A = Σ m_i (x*_i − c) r_iᵀ
Q = rotation part of A
Δx_i = (Q r_i + c) − x*_i
```

Q is found with the iterative method of Müller et al. 2016, warm-started from the previous frame's rotation.
It is robust when A is degenerate, for example for flat shapes.

**Sparse SDF (§5.1).** Every rigid particle stores φ (signed distance to the surface, negative inside) and the
outward gradient ∇φ, in local space. For meshes, both come from the voxelization:

1. A voxel is inside when a ray from its center crosses the mesh an odd number of times (ray-triangle test of
   chapter 3).
2. φ is the distance to the nearest outside voxel.

The surface is placed half a radius outside the outer layer of particles, so the outer layer has `|φ| < r`
(boundary particles) and deeper particles are interior.

**Contacts between rigid particles of different bodies:**

- Both interior: the normal is the gradient of the particle closer to its surface, and the depth is
  `min(|φ_i|, |φ_j|)` (eq. 17-19). Deeply interlocked bodies are pushed apart along their minimum
  translation instead of being trapped.
- A boundary particle acts as a one-sided sphere: when the other particle lies behind the surface, its
  direction is reflected about ∇φ, so it is pushed outward (eq. 20).

**Shock propagation (§5.2).** For contacts only, `w_i* = w_i e^(k h_i)`, where h is the height above the
ground. Lower particles look heavier, so a stack converges in a few iterations. Measured on a 10-cube stack:
compression goes from 6.9 cm to 3.2 cm with k = 3.

---

## 7. Friction (paper §6)

After the penetration correction of a contact of depth d, with n the contact normal:

```
Δx⊥ = [(x*_i − x_i) − (x*_j − x_j)]⊥n                                  (eq. 23)

f = Δx⊥                          if |Δx⊥| < μs d     (static: all sliding removed)
f = Δx⊥ min(μk d / |Δx⊥|, 1)     otherwise           (kinetic, Coulomb)  (eq. 24)

Δx_i −= w_i/(w_i+w_j) f      Δx_j += w_j/(w_i+w_j) f
```

Because friction acts on positions, it can cancel the drift that the constraint projection itself causes. That
is why sand piles stay steep. The same model is used against kinematic shapes, taking the shape's own motion
into account.

---

## 8. Fluids (paper §7)

The kernels have support radius `h = 2 × fluid rest spacing`: poly6 for densities, the gradient of the spiky
kernel for corrections.

```
ρ_i  = Σ_fluid W(x_i − x_j) + s Σ_solid W(x_i − x_j)          (eq. 27)
C_i  = max(ρ_i / ρ0 − 1, 0)                                   (eq. 26, unilateral)
λ_i  = −C_i / (Σ_k |∇_k C_i|² + ε D_ref)    D_ref computed once on a filled reference lattice
Δx_i = (w_i / w_ref) / ρ0 Σ_j (λ_i + λ_j) ∇W(x_i − x_j)
```

The factor `w_i / w_ref` is the mass weighting of §7.1.1. Lighter particles move more, which gives buoyancy for
rigid bodies and layering of immiscible fluids. w_ref is the largest fluid inverse mass, as advised in the paper.

Fluid-solid contacts use the fluid rest distance (§7.1).

After the velocity update, every fluid particle gets:

- **XSPH viscosity:** `v_i += c Σ (v_j − v_i) W / ρ_i`.
- **Vorticity confinement:** `ω_i = Σ (1/ρ_j)(v_j − v_i) × ∇_j W`, `N = ∇|ω| / |∇|ω||`, `f = ε h (N × ω)`.
- **Cohesion and surface tension (Akinci 2013):** within the same phase only. They use the normalized cohesion
  spline and the curvature term `−(n_i − n_j)`, with strengths expressed as accelerations in units of g.

`fluidGravityScale` scales gravity for fluid particles only: a negative value makes a fluid rise, the starting
point of a gas (§7.2).

---

## 9. Distance constraints (paper §8)

**Distance constraints.**

```
C = |x_i − x_j| − L
Δx_i = −w_i/(w_i+w_j) C n
Δx_j = +w_j/(w_i+w_j) C n
```

The stiffness is converted per iteration, `k' = 1 − (1 − k)^(1/iterations)`, so it does not depend on the
iteration count.

**Self-collision.** With the `SelfCollide` and `SelfCollideFilter` phase flags, particles of the same group
collide unless they are closer than the contact distance in the rest pose (neighbors in a cloth or a rope).

**Tearing.** A distance constraint breaks when it is stretched beyond `tearRatio × L` (its `broken` flag is
set and it is skipped from then on).

Cloth, ropes and inflatables are built from these constraints by the simulations themselves.
