# Cloth with real tearing (paper §8)

Scene: `Scenes/ClothShowcase.unity`. It has three pieces of cloth, all simulated by the shared unified solver:

- a **flag** on a pole in gusty wind (aerodynamics, tethers),
- a **tartan curtain** hanging from a beam, that tears when it is pulled, cut or ripped (real mesh splitting),
- a **sheet** falling on a sphere and a box (collisions, self-collision, draping).

`Scenes/08_ClothFlag.unity` is the simpler starting scene (flag and sheet, no tearing).

## Controls

| Input | Action |
|---|---|
| Left mouse drag | Grab and pull a particle (pull the curtain hard to tear it) |
| Hold **C** + move the mouse | Blade: cut the curtain or the sheet along the mouse path |
| **T** | Rip the curtain from the bottom up (always the same, good for recording) |
| **J** | Switch the distance constraints between Gauss-Seidel and Jacobi (the paper's solver) |
| **W** | Wind on / off |
| **1** to **4** | Camera shots: wide, flag, curtain, sheet |
| **R** | Reset the scene |
| **H** | Hide the text (clean recording) |
| Space / P | Radial impulse under the mouse / pause |
| Right / middle mouse, wheel, F | Orbit / pan, zoom, reset the camera |

## Cloth model

Cloth is a grid of particles linked by distance constraints (the library's `DistanceConstraint`, solved with
per-iteration stiffness `k' = 1 − (1 − k)^(1/n)`):

- **stretch** along the grid edges, **shear** along the diagonals, **bend** between particles two apart.

**Gauss-Seidel vs Jacobi.** The paper solves every constraint group with Jacobi iterations and averaging
(eq. 12-13), which suits the GPU. On the CPU the library defaults to Gauss-Seidel for distance constraints.
Measured on the flag in the wind (3 sub-steps × 4 iterations): **22 % maximum stretch with Jacobi, 7 % with
Gauss-Seidel**. Press J in the scene to see the difference.

**Tethers** (`ClothTethers`, long range attachments): `C = |x − x_anchor| − L ≤ 0`. They only pull when a
particle is farther than L from its closest pinned particle, which stops the flag from over-stretching under
the wind.

**Aerodynamics** (`AerodynamicsExtension`, Keckeisen et al. 2004). Each triangle is a thin airfoil. With
`v_rel = v_triangle − v_wind` and n oriented against the flow:

```
drag = −k_d A (n·v_rel) v_rel
lift = −k_l A (n·v_rel) |v_rel| (n − (n·v̂) v̂)
```

The force is shared between the three vertices as a velocity change. The wind is `SolverParams.wind`; the
showcase varies it with Perlin noise (gusts and direction changes).

## Real tearing (`ClothTearing`)

The paper tears cloth by removing constraints (§10, fig. 18). Removing constraints alone leaves the triangles
connected: the rendered mesh never opens. `ClothTearing` keeps the topology of the mesh and splits it.

**Data.** The grid vertices are the *original* vertices. Each triangle stores, for each corner, its original
vertex (fixed) and the particle it currently uses (changes when a vertex splits). Each distance constraint is
classified once:

| Constraint | Grid offset | Role in the topology |
|---|---|---|
| Edge | (1,0), (0,1), and the triangles' diagonal (1,−1) | a mesh edge, shared by 1 or 2 triangles |
| Cross | the other diagonal (1,1) | crosses the quad's diagonal edge |
| Bend | (2,0), (0,2) | spans two edges through a middle vertex |

Each constraint also remembers which triangle gives each of its endpoints.

**Tearing an edge.** When an edge tears, the shear and bend constraints that rely on it break too (otherwise
they would hold the two sides together).

**Splitting a vertex.** Around each vertex of a torn edge, the triangles are grouped with a union-find: two
triangles are in the same group if they share an intact edge through the vertex. Every group gets its own
particle: the largest keeps the existing one, and the others get a copy with the same position, velocity, mass
and rest position (so the two sides of the tear don't collide with each other). Then every constraint around
the vertex is re-attached to the particle on its own side, through the triangle it remembers.

**Detached triangles.** A triangle that lost two of its edges hangs by one edge only. The corner between the
two torn edges has no constraint left, so it is left out of the mesh.

**Crack propagation.** The solver does not break constraints by itself (its tear ratio is 0). Each frame,
`ClothTearing` tears only the 3 most stretched edges above `tearRatio × L`. The stress then moves to the tip of
the crack, which runs through the cloth, instead of every over-stretched edge breaking at once and shattering
the area.

**Cutting.** The blade snaps the mouse path to the closest vertices and tears the grid edges of a path between
them (a diagonal step uses the triangles' diagonal). Cutting along mesh edges gives a clean slit: no triangle
is lost.

**Rendering.** The mesh is rebuilt when the topology changes. Every particle (copies included) is a vertex with
the UV of its original grid vertex, so the texture stays continuous on both sides of a tear. The mesh is
double-sided (a second set of vertices with flipped winding) and its normals are computed by hand.

## Files

| File | Role |
|---|---|
| `Scripts/ClothBuilder.cs` | `CreateCloth` (particles, constraints, triangles) and `AddClothTethers`. |
| `Scripts/ClothTearing.cs` | Topology, vertex splitting, crack propagation and cutting. |
| `Scripts/ClothTethers.cs` | Tether constraint and its solver extension. |
| `Scripts/ClothAerodynamics.cs` | Aerodynamics extension, and the `ClothAerodynamics` component for the `UnifiedWorld`. |
| `Scripts/ParticleCloth.cs` | Component: a cloth rectangle with pin modes, tethers, tearing, texture. Builds its mesh. |
| `Demo/ClothShowcase.cs` | The showcase scene: layout, wind gusts, rip, camera shots, on-screen text. |
| `Demo/ClothCutter.cs` | The blade tool (hold C). |
| `Demo/ClothTextures.cs` | Procedural textures (flag, tartan, gingham, floor). |
| `Demo/DemoClothFlag.cs` | The simpler starting scene. |
| `Tests/ClothTests.cs` | Stretch under 10 %, tethers, a cut splits the cloth in exactly two pieces (10 edges, 11 split vertices), a partial cut keeps one piece, a cut piece falls while the rest hangs. |

## Recording tips

1. Play the scene, press **H** to hide the text if you want a clean shot.
2. Wide shot (**1**): let the sheet fall and the flag fly for a few seconds.
3. Curtain shot (**3**): press **T** for the rip, or hold **C** and draw a cut. Press **R** to start again.
4. Flag shot (**2**): press **J** a few times to show Jacobi stretching the flag.
