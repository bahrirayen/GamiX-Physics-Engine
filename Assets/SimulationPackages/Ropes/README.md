# Ropes (paper §8, fig. 1)

Crates hanging from ropes pinned to a beam, and a rope tied between two bodies. Drag a crate with the left
mouse button.

A rope is a chain of particles with:

- **stretch** constraints between neighbors `(i, i+1)`,
- **bend** constraints between particles two apart `(i, i+2)`, softer, so the rope can curve,

all solved as the library's distance constraints `C = |x_i − x_j| − L`. The ends are pinned (inverse mass 0)
or tied to a particle rigid body with `ParticleFactory.Attach`, which links the end particle to the closest
particle of the body. Because the rope and the crate are solved in the same loop, the crate swings and pulls
on the rope (two-way coupling). Rope particles use the `SelfCollideFilter` phase flag, so a rope collides with
itself but not with its direct neighbors.

## Contents

| File | Role |
|---|---|
| `Scripts/RopeBuilder.cs` | `RopeHandle` and `CreateRope`. |
| `Scripts/ParticleRope.cs` | Component: a rope between two points or two rigid bodies, rendered as a tube. |
| `Demo/DemoRopes.cs` | The beam, the crates and the ropes. |
| `Scenes/09_Ropes.unity` | The scene. |
