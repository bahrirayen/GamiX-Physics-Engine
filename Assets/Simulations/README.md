# Simulations

Each team member builds one simulation from the paper here, using only the shared library in
`Assets/GamiXPhysics`.

## Conventions

- Each member has a folder: `Oubay/`, `Rayen/`, `Yassine/`. Only its owner edits it (see
  [CONTRIBUTING.md](../../CONTRIBUTING.md)).
- Put your scene, scripts (namespace `GamiX.Sim.<Name>`), prefabs and materials in your folder. Its assembly
  definition already references `GamiX.Physics`, `GamiX.Physics.Demos` (camera, particle grabber) and
  `Unity.InputSystem`.
- Do not edit `Assets/GamiXPhysics` for a single simulation. If the library is missing something, add it there in
  a general form, with a test, through a pull request.
- Never use Unity's `Rigidbody`, `Collider` or `Physics.*`. The `ProjectRulesTests` test fails if you do.
  `GameObject.CreatePrimitive` adds a Collider: remove it, or build the visual from a MeshFilter and a MeshRenderer.

## Minimal setup

1. Create an empty GameObject with `UnifiedWorld` and `ParticleRenderer`.
2. Add a `ParticleCollider` (Plane) for the ground.
3. Add the bodies you need: `ParticleRigidBody`, or your own components derived from `ParticleBody`.
4. Put an `OrbitCamera` and a `ParticleGrabber` (from `GamiX.Physics.Demos`) on the camera for interaction.
5. For logic, subscribe to `UnifiedWorld.BeforeStep` / `AfterStep`. The solver state is public: `positions`,
   `velocities`, `invMass`, `Rigids`, `DistanceConstraints`...
6. Physics specific to your simulation (new constraints, forces, velocity effects) goes in a `SolverExtension`
   in your folder. The README of the repository shows an example.
