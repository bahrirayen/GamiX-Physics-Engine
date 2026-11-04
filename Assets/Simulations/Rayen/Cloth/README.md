# Cloth (paper §8)

A flag in the wind attached to a pole, and a sheet falling on a sphere. Drag the cloth with the left mouse
button.

Cloth is a grid of particles linked by distance constraints (the library's `DistanceConstraint`, solved with
per-iteration stiffness `k' = 1 − (1 − k)^(1/n)`):

- **stretch** along the grid edges, **shear** along the diagonals, **bend** between particles two apart.

This package adds two extensions to the solver:

**Tethers** (`ClothTethers`, long range attachments): `C = |x − x_anchor| − L ≤ 0`. They only pull when a
particle is farther than L from its closest pinned particle, which stops the cloth from over-stretching under
its own weight.

**Aerodynamics** (`AerodynamicsExtension`, Keckeisen et al. 2004). Each triangle is a thin airfoil. With
`v_rel = v_triangle − v_wind` and n oriented against the flow:

```
drag = −k_d A (n·v_rel) v_rel
lift = −k_l A (n·v_rel) |v_rel| (n − (n·v̂) v̂)
```

The force is shared between the three vertices as a velocity change. The wind is `SolverParams.wind`.

**Tearing.** With `tearRatio > 1`, a constraint breaks when stretched beyond `tearRatio × L`. The mesh itself is
not split yet: splitting the vertices along the tear is the natural next step.

## Contents

| File | Role |
|---|---|
| `Scripts/ClothBuilder.cs` | `CreateCloth` (particles, constraints, triangles) and `AddClothTethers`. |
| `Scripts/ClothTethers.cs` | Tether constraint and its solver extension. |
| `Scripts/ClothAerodynamics.cs` | Aerodynamics extension, and the `ClothAerodynamics` component to put next to the `UnifiedWorld`. |
| `Scripts/ParticleCloth.cs` | Component: a cloth rectangle with pin modes, tethers, tearing. Generates and deforms its mesh. |
| `Demo/DemoClothFlag.cs` | The flag and the sheet. |
| `Scenes/08_ClothFlag.unity` | The scene. |
| `Tests/ClothTests.cs` | Stretch stays under 10%, tethers hold their maximum length. |
