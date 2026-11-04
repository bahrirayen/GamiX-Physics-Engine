# GamiX Physics Engine

A physics engine written from scratch in C# for Unity, without any of Unity's physics
(no `Rigidbody`, no `Collider`, no `Physics.*`).

It contains two engines that share the same math, geometry and collision code:

- **Unified particle solver**: the core of *Unified Particle Physics for Real-Time Applications*
  (Macklin, Müller, Chentanez, Kim, NVIDIA, SIGGRAPH 2014, the "FleX" paper). Everything is made of particles
  coupled by position-level constraints. The library has what every simulation of the paper shares: the
  solver loop (Algorithm 1), contacts and friction, the fluid density solver, rigid bodies by shape matching,
  distance constraints, and an extension API through which each simulation adds its own physics.
- **Classic rigid body engine**: Baraff-style rigid bodies with inertia tensors computed from the geometry,
  a broad phase, an exact narrow phase (SAT) and an impulse-based contact solver.

Course project: *Moteur Physique pour les jeux vidéo*, 5GamiX, Esprit, 2026-2027.

Team: Oubay Bouzekri, Rayen Bahri, Yassine Hemdene. Each member builds one simulation of the paper with the
library, in `Assets/Simulations/<Name>/`.

---

## Contents

1. [Quick start](#quick-start)
2. [Architecture](#architecture)
3. [Modules](#modules)
4. [The unified solver step by step](#the-unified-solver-step-by-step)
5. [Using the library in a simulation](#using-the-library-in-a-simulation)
6. [Design choices and differences from the paper](#design-choices-and-differences-from-the-paper)
7. [Performance](#performance)
8. [Tests](#tests)

More detail on the math is in [docs/TECHNICAL.md](docs/TECHNICAL.md).

---

## Quick start

1. Open the project with **Unity 6000.6.1f1** (URP, new Input System).
2. Menu **GamiX Physics > Set Code Optimization To Release**. The simulation runs several times faster in the
   editor in Release mode.
3. Menu **GamiX Physics > Create Demo Scenes**. This writes the library's demo scenes to
   `Assets/GamiXPhysics/Demos/Scenes`.
4. Open a scene and press Play.

| Scene | Shows | Paper / course |
|---|---|---|
| 01_RigidBodies | Pyramid and tower of boxes, bouncing spheres, a spinning T-handle (Dzhanibekov effect) | Ch. 2 part 2, Ch. 4 |
| 02_BroadPhase | 1000 spheres in a box: brute force vs sweep and prune vs uniform grid (keys 1/2/3) | Ch. 4, apps 7-8 |
| 03_MassSpring | The same spring chain with explicit Euler, semi-implicit Euler, Verlet and RK4 | Ch. 2 part 1 |

Controls: right mouse to orbit, middle mouse to pan, wheel to zoom, F to reset the camera. In 01_RigidBodies,
left click shoots a ball and right click pushes a body. In particle scenes, the `ParticleGrabber` on the camera
lets the left mouse drag a particle; Space gives a radial impulse and P pauses.

---

## Architecture

```
Assets/GamiXPhysics/
├── Runtime/                    assembly GamiX.Physics (the library)
│   ├── Math/                   VMath, Quat, Mat3, Mat4, Numerics          (Ch. 1, Ch. 3)
│   ├── Geometry/               primitives, distances, ray casting, BV     (Ch. 3, Ch. 4)
│   ├── Collision/              narrow phase, broad phases, time of impact (Ch. 4)
│   ├── Integration/            generic ODE solver, point integrators      (Ch. 2 part 1)
│   ├── MassSpring/             force-based particle system                (Ch. 2 part 1)
│   ├── RigidBodies/            Baraff rigid bodies + contact solver       (Ch. 2 part 2, Ch. 4)
│   ├── Unified/                the FleX unified particle solver           (paper)
│   │   ├── Constraints/        distance constraints, rigid groups, particle meshes
│   │   └── Builders/           fluid and granular blocks, rigid bodies, mesh voxelization
│   ├── Components/             MonoBehaviours to author scenes
│   └── Rendering/              instanced particle rendering, helper meshes
├── Demos/                      assembly GamiX.Physics.Demos (course scenes, camera, particle grabber)
├── Editor/                     demo scene generator
└── Tests/EditMode/             NUnit tests (Window > General > Test Runner)
Assets/Simulations/             Oubay/, Rayen/, Yassine/: one simulation per team member
```

Dependencies only go downward: `Math` ← `Geometry` ← `Collision` ← `RigidBodies` / `Unified` ← `Components`.
The simulations depend on the library, never the other way around.
The core (`Math` to `Unified`) does not depend on `MonoBehaviour`, so it runs outside Play mode and outside
Unity. The tests run it headless.

Unity is used for **display and input only**. `UnityEngine.Vector3` is used as an (x, y, z) container, and
dot and cross products, lengths, normalization, quaternions, matrices, inverses and rotations are all written
in `VMath`, `Quat`, `Mat3` and `Mat4`. Unity's `Quaternion` and `Matrix4x4` only appear when a pose is handed
to the renderer.

---

## Modules

### Math (`GamiX.Physics`)
- `VMath`: dot, cross, length, normalization, projection, reflection, orthonormal basis, back-face test.
- `Quat`: axis-angle (slide 55) and the reverse rules, Hamilton product (slide 53), conjugate and inverse
  (slide 57), rotation `q·P·q⁻¹` (slide 58), conversion to and from a matrix (slide 59), Slerp (slide 61),
  integration `dq/dt = ½ ω q`.
- `Mat3`: product, transpose, determinant, adjugate inverse, skew matrix `ω*` (Ch. 2, slide 41), outer
  product, Gram-Schmidt re-orthonormalization (Ch. 2, slide 36), polar decomposition (rotation extraction for
  shape matching).
- `Mat4`: homogeneous transforms, TRS, applying one transform to every vertex (Ch. 1, slide 13).
- `Numerics`: quadratic and cubic roots (Ch. 3, slides 30-32), bisection (Ch. 4, slide 12), 2×2 solve.

### Geometry (`GamiX.Physics.Geometry`)
`Ray3D`, `Line`, `Segment`, `Plane3D` (`<n, D>`), `Sphere`, `Aabb`, `Obb`, `Triangle`, `Capsule`, `Cylinder`.

- `Distance`: point-line, line-line by the 2×2 system of slide 17, segment-segment, point-plane, closest point
  on a triangle, barycentric coordinates, intersection of three planes.
- `Raycast`: ray against plane, sphere, AABB (slab method), OBB (in object space), triangle (barycentric),
  capped cylinder, capsule, mesh.
- `BoundingVolumes`: the three bounding sphere methods of Ch. 4, slide 23 (AABB center, centroid, Ritter).
- `Overlap`: sphere, AABB and OBB tests, including SAT on 15 axes.

### Collision (`GamiX.Physics.Collision`)
- `NarrowPhase`: contact manifolds (point, normal, depth) for sphere-sphere, sphere-plane, sphere-OBB,
  OBB-plane, and OBB-OBB (SAT, then incident face clipping for face contacts or the closest points of the edges
  for edge-edge contacts).
- `IBroadPhase` (Ch. 4, slide 72) with `BruteForceBroadPhase`, `SweepAndPruneBroadPhase` (persistent order,
  insertion sort, automatic axis) and `UniformGridBroadPhase` (spatial hashing, no duplicate pairs).
  `SweepAndPrune1D` is the interval version of slides 58-61.
- `TimeOfImpact`: interpolation, bisection, continuous sphere-plane and sphere-sphere detection, number of
  sub-steps (Ch. 4, applications 1-2).

### Integration and mass-spring systems
- `OdeSolver` works on any `IOdeSystem` (flat state vector, Ch. 2, slides 30-33) with explicit Euler, midpoint or
  RK4. Every RK4 stage re-evaluates the forces at the intermediate state, which is what makes it 4th order.
- `PointIntegrators`: explicit and semi-implicit Euler, Verlet and RK4 for a single point, plus the analytic
  free fall with viscous drag for error measurements.
- `MassSpringSystem`: the particle system of slide 33. Particles go to the ODE solver as a 6N vector, and
  forces are a list of objects (`GravityForce`, `ViscousDragForce`, `SpringForce`, `AnchorSpringForce`,
  `WindForce`). Particles bounce on planes.

### Rigid bodies (`GamiX.Physics.RigidBodies`)
- `RigidBody`: state `X = (x, q, P, L)` with derived `v = P/M`, `I⁻¹(t) = R I_body⁻¹ Rᵀ` and `ω = I⁻¹ L`
  (Ch. 2, slides 37-41). Forces at a point create torque `τ = (r − x) × F`. Impulses, point velocities and
  kinetic energy are available.
- `InertiaTensor`: box, solid and hollow sphere, solid and hollow cylinder, capsule, any set of point masses
  (slide 31), and the parallel axis theorem.
- `RigidBodyWorld`: semi-implicit Euler, explicit Euler or RK4 integration, broad phase, narrow phase, and an
  iterative impulse solver using Baraff's effective mass
  `j = −(1+e) v_rel / (1/mA + 1/mB + n·((I_A⁻¹(rA×n))×rA) + n·((I_B⁻¹(rB×n))×rB))`, with Coulomb friction,
  Baumgarte stabilization, restitution threshold and sleeping. It also has `RigidSpring` force generators
  (springs that can break), ray casting into the scene, and momentum and energy diagnostics.

### Unified particle solver (`GamiX.Physics.Unified`)
`UnifiedSolver` implements the paper's solver. See the next section. `SolverExtension` is the extension point
for simulations (see [Using the library in a simulation](#using-the-library-in-a-simulation)).

### Unity components (`GamiX.Physics.Components`)
| Component | Role |
|---|---|
| `UnifiedWorld` | Owns the `UnifiedSolver` and steps it at a fixed rate. Exposes every `SolverParams` setting in the Inspector. |
| `ParticleCollider` | Plane, box, sphere or capsule. Can be a container. Follows its Transform, so it can be animated. |
| `ParticleBody` | Base class of every particle object: derive from it and create particles in `Create`. |
| `ParticleRigidBody` | A box, sphere or any closed mesh (voxelized) kept rigid by shape matching. Drives its Transform. |
| `ParticleRenderer` | Draws every particle with GPU instancing, colored per body. |
| `RigidWorld`, `RigidBodyComponent` | The classic rigid body engine. `DisplayMode.ManualVertices` moves the mesh vertices with our own `Mat4` instead of setting the Transform. |

---

## The unified solver step by step

`UnifiedSolver.Step(dt)` splits the frame into sub-steps. Each sub-step follows **Algorithm 1** of the paper:

| Algorithm 1 | Code |
|---|---|
| 1-5: apply forces, predict `x* = x + Δt v`, mass scaling `m* = m e^(−kh)` | `SolverExtension.ApplyForces`, `PredictParticle` |
| 6-9: find neighbors and solid contacts | `HashGrid` (counting sort), `BuildParticleContacts`, `BuildShapeContacts` |
| 10-15: pre-stabilization of contacts on `x` and `x*` (§4.4) | `stabilizationIterations` loop |
| 16-22: constraint groups solved in parallel, averaged `Δx_i = ω/n Σ ∇C λ` (eq. 12-13), applied group by group | density, shape contacts, particle contacts, distance, `SolverExtension.SolveConstraints`, shape matching |
| 23-28: `v = (x* − x)/Δt`, internal forces, sleeping (eq. 14) | `ApplyFluidInternalForces`, `SolverExtension.UpdateVelocities`, `EndSubStep` |

What the shared object types are made of:

| Object | Particles + constraints | Paper |
|---|---|---|
| Rigid body | shape matching `Δx_i = (Q r_i + c) − x*_i`, Q from the polar decomposition of `A = Σ m (x* − c) rᵀ`; per-particle SDF (φ, ∇φ) from voxelization | §5, eq. 15-16, §5.1, eq. 17-20 |
| Stiff stacks | shock propagation, contact mass scaling `s = e^(−kh)` | §5.2, eq. 21 |
| Plastic solids | rest offsets creep toward the deformed shape above a threshold | §5.3 |
| Granular | contacts + friction at the position level (static below `μs·d`, kinetic limited by `μk·d`) | §6, eq. 22-25 |
| Fluid | unilateral density constraint `ρ/ρ0 − 1 ≤ 0`, solids counted with weight `s`, mass-weighted corrections give buoyancy and multi-phase fluids, XSPH viscosity, vorticity confinement, cohesion and surface tension | §7, eq. 26-27 |
| Distance link | `C = ‖x_i − x_j‖ − L` with per-iteration stiffness and optional tearing, self-collision filtered by rest distance | §8 |

---

## Using the library in a simulation

Each team member works in their own folder and branch. Read [CONTRIBUTING.md](CONTRIBUTING.md) first. With components, no code is
needed for a basic scene:

1. Add an empty GameObject with `UnifiedWorld` and `ParticleRenderer`.
2. Add a `ParticleCollider` with shape Plane for the ground.
3. Add a `ParticleRigidBody` on a cube, or your own `ParticleBody` components.
4. Press Play and tune the `UnifiedWorld` settings in the Inspector.

From code:

```csharp
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

var solver = new UnifiedSolver(new SolverParams { radius = 0.05f, substeps = 2, iterations = 4 });
solver.AddShape(new KinematicPlane());                                              // ground
solver.AddShape(new KinematicBox { position = new Vector3(0, 1, 0),
                                   halfExtents = new Vector3(1, 1, 0.5f), inverted = true }); // tank
ParticleFactory.CreateFluidBlock(solver, new Vector3(-0.5f, 0.5f, 0), new Vector3(0.8f, 0.8f, 0.8f));
RigidGroup crate = ParticleFactory.CreateRigidBox(solver, new Vector3(0.5f, 1.5f, 0), Quat.Identity,
                                                  new Vector3(0.3f, 0.3f, 0.3f), particleMass: 0.3f);
for (int frame = 0; frame < 600; frame++)
    solver.Step(1f / 60f);
Debug.Log($"crate at {crate.center}, rotation {crate.rotation}");
```

The classic rigid body engine works the same way:

```csharp
using GamiX.Physics.RigidBodies;

var world = new RigidBodyWorld();
world.AddBody(RigidBody.CreateStatic(new PlaneShape(), Vector3.zero, Quat.Identity));
RigidBody box = world.AddBody(new RigidBody(new BoxShape(new Vector3(0.5f, 0.5f, 0.5f)), 2f,
                                            new Vector3(0, 3, 0), Quat.FromAxisAngleDeg(Vector3.up, 30f)));
box.AddForceAtPoint(new Vector3(0, 0, 50), box.position + new Vector3(0.5f, 0, 0)); // force + torque
world.Step(1f / 60f);
```

`UnifiedWorld.BeforeStep` and `AfterStep` are hooks for gameplay logic, such as moving kinematic shapes,
reading contacts or triggering fracture.

Physics that only one simulation needs goes in a `SolverExtension`, in the simulation's folder. Example: a
constraint that keeps particles above a height.

```csharp
public sealed class FloorConstraint : SolverExtension
{
    public float height;

    public override bool SolveConstraints(int iteration)
    {
        UnifiedSolver s = Solver;
        bool any = false;
        for (int i = 0; i < s.Count; i++)
        {
            if (!s.active[i] || s.invMass[i] == 0f) continue;
            float c = s.predicted[i].y - height;          // C(x) >= 0
            if (c >= 0f) continue;
            s.AddCorrection(i, new Vector3(0f, -c, 0f)); // averaged with the other corrections (eq. 12)
            any = true;
        }
        return any;
    }
}

solver.GetOrAddExtension<FloorConstraint>().height = 0.5f;
```

The other hooks are `ApplyForces` (before the prediction), `UpdateVelocities` (after the velocity update) and
`EndSubStep`. Neighbors (`GetNeighbors`, `NeighborAt`), densities, the SPH `Kernels`, the `Grid` and
`ParallelForRange` are public for extensions.

---

## Design choices and differences from the paper

| Topic | Choice | Why |
|---|---|---|
| Hardware | CPU, with `Parallel.For` on the particle-centric passes (prediction, neighbors, contacts, density) | The paper targets CUDA. Jacobi averaging is what makes these passes safe to parallelize. |
| Distance constraints | Gauss-Seidel by default (`gaussSeidelDistance`), Jacobi as an option | The paper uses Jacobi for GPU parallelism. On the CPU, Gauss-Seidel brings the stretch of a hanging cloth from 28% down to 5% at the same cost. |
| Friction sign (eq. 24-25) | `Δx_i = −w_i/(w_i+w_j) Δx⊥`, `Δx_j = +w_j/(w_i+w_j) Δx⊥` | As printed, eq. 24 has no minus sign and eq. 25 applies the mass weight twice. The implemented form removes the relative tangential motion. Measured stopping distance: 1.52 m against 1.53 m analytic. |
| One-sided normals (eq. 20) | The other particle's direction is reflected about the boundary particle's SDF gradient when it lies behind the surface | This is how the paper's wording ("when a particle lies in the negative half-space") is read. |
| Density denominator | Computed per particle, plus 1% of the filled-lattice reference value as relaxation | §7.1.1 precomputes it on the reference lattice. Under impact compression the real gradients are larger, and the fixed value made a dam break explode. The lattice still sets the rest density ρ0. |
| Solid weight `s` in the density (eq. 27) | 0.5 by default | Measured: a box at 0.3× the fluid density floats with about a third of its height above the surface. |
| Fluid friction | Separate multiplier, 0 by default | With wall friction, a thin fluid layer sticks to the walls and the Rayleigh-Taylor instability never develops. |
| Vorticity, cohesion, surface tension | SPH volume `1/ρ_j` in the curl, confinement scaled by `ε·h`, Akinci cohesion and surface tension given as accelerations in units of g | The parameters keep their meaning whatever the particle size and time step. |
| Rigid orientation | Unit quaternion instead of the 3×3 matrix R | No drift to correct. Gram-Schmidt (slide 36) is still in `Mat3.Orthonormalize`. |

Known limitations (the same as in the paper's §11): the particle radius is fixed per solver, buoyancy does not
match real submersion depths exactly, and shape matching converges more slowly for large rigid bodies.

---

## Performance

Measured headless, Release build, 16-core CPU, 2 sub-steps × 4 iterations per frame:

| Scene | Particles | ms per frame |
|---|---|---|
| Fluid in a tank | 4,275 | 6.2 (27 on one thread) |
| Fluid + rigid boxes (buoyancy) | 901 | 1.7 |
| 6 rigid boxes | 216 | 0.5 |

In the Unity editor (Mono, Release code optimization), a dam break with 1,385 fluid particles and two rigid
boxes takes about 13 ms per frame. Mono is slower than the .NET runtime used for the table above, so use the
Release code optimization (see Quick start).

Optimizations:
- The hash grid is built by counting sort, with no allocation per frame.
- Neighbor lists are found once per sub-step and reused by every constraint.
- Particle state is stored as arrays of structs.
- Delegates are cached for the parallel loops.
- Contacts are particle-centric: each particle stores its own contact list and computes its own correction, so
  contact generation and solving run in parallel without locks. Both particles of a pair compute the same
  correction (canonical pair order), so the result does not depend on thread scheduling.
- The SPH kernels are inlined in the density, viscosity and cohesion loops.
- Shape matching runs one rigid group per thread.
- The per-iteration stiffness of distance constraints is cached.
- Broad phases are interchangeable, and SAP uses temporal coherence.
- Rigid bodies sleep, and so do particles (eq. 14).

---

## Tests

`Assets/GamiXPhysics/Tests/EditMode` holds 34 NUnit tests (Window > General > Test Runner > EditMode). They
compare the library with the worked examples of the course:

- Triangle normal, slide 10.
- Back-face angle of 93.68°, slide 12.
- Quaternion table, slide 56.
- Point-line distance, slide 14.
- Line-line distance 4√3/3, slides 19-20.
- Plane distance, slide 26.
- Three planes, slide 28.
- Polynomial roots, slides 31-32.
- Sweep and prune intervals, Ch. 4, slide 61.
- Continuous detection at x = 4.85, Ch. 4, application 2.
- Inertia tensors, Ch. 2, slide 33.

They also check physical invariants:

- RK4 accuracy compared with Euler.
- Energy and momentum conservation in elastic collisions.
- A stable box stack.
- Rest density of a fluid.
- Rigidity under shape matching.
- Coulomb friction distance.
- Length of a chain of distance constraints.
- The order of the extension hooks in a step.
