# Yassine simulation: fluids

Owner: Yassine Hemdene

Simulation from the paper: **fluids and fluid-solid coupling** (section 7 of *Unified Particle Physics for
Real-Time Applications*). Fluid particles are kept at the rest density by the unilateral density constraint
(eq. 26). Solids count in the fluid density with weight `s` (eq. 27). Mass-weighted corrections give buoyancy and
multi-phase fluids (§7.1.1).

## Running it

1. Menu **GamiX Physics > Yassine > Create Fluid Scene**. This writes `Scenes/Fluids.unity` in this folder.
2. Open it and press Play.

| Key | Action |
|---|---|
| 1 / 2 / 3 / 4 | Dam break / Buoyancy / Rayleigh-Taylor / Fountain |
| R | Restart the scenario |
| V | Show the water as a smooth surface or as particles |
| C | Particle view: color by phase, speed or density (`rho/rho0`) |
| B | Drop a crate (the density cycles: 0.2, 0.5, 2.0 times water) |
| E | Turn the fountain's emitter on or off |
| H | Hide the panel |
| Left drag / Space / P | Grab a particle / radial splash / pause (library `ParticleGrabber`) |
| Right drag / middle drag / wheel / F | Orbit / pan / zoom / reset the camera |

The control panel on the right works like the cloth demo's:

- **Scenario**: buttons for the four scenarios.
- **Fluid**: live sliders for viscosity, cohesion, surface tension, vorticity confinement ε, and solid weight `s`.
- **Emitter** (fountain only): jet speed.
- **Solver**: sub-steps, iterations, relaxation ω, gravity.
- **Buttons**: surface or particle view, color mode, drop crate, splash, pause, reset, emitter on/off.
- **Max compression** (`max rho/rho0 - 1`): a bar chart of the last 6 s against a 5% reference line. Bars over the
  line turn orange.
- **Readouts**: mean density, wave front, mean height of each phase, emitter counts, and how far each crate sits
  under water next to Archimedes' prediction. Then particles, solver ms and FPS.

The sliders edit the solver's `SolverParams` directly, so changes take effect on the next frame.

## Scenarios

| Scenario | What happens | Paper |
|---|---|---|
| Dam break | A 1.2 m water column collapses, hits an obstacle, and lifts a light crate (0.4× water) off the floor. Vorticity confinement keeps the splash lively. | §7, eq. 26 |
| Buoyancy | Crates at 0.2×, 0.4× and 2× the density of water. The two light crates float, the light one higher, and the steel crate sinks. | §7.1.1, eq. 27 |
| Rayleigh-Taylor | A fluid 3× heavier sits on top of a light one. The interface breaks into fingers and the phases swap. Cohesion acts within a phase only, so the interface stays sharp. | §7.1.1 |
| Fountain | A nozzle fills an empty basin. The floating balls rise with the water. An overflow outlet on the right wall removes the excess, so the level and particle count settle. | §7 (emitters) |

## Code

| File | Role |
|---|---|
| `Scripts/FluidScenarios.cs` | Builds each scenario in a `UnifiedSolver`, with tuned `SolverParams`. Pure physics: no MonoBehaviour, so the tests run it headless. |
| `Scripts/FluidEmitter.cs` | `SolverExtension` that adds a disc of particles on the fluid lattice each time the jet travels one rest spacing, and removes particles that enter the drain box. It works in `EndSubStep`, between sub-steps. |
| `Scripts/FluidStats.cs` | Measurements: density ratio, mean height, wave front, free-surface height, submerged fraction. |
| `Scripts/FluidSurfaceMesher.cs` | Water surface for display. Each particle splats a smooth bump on a grid, Surface Nets extracts the iso-surface (one vertex per crossed cell, one quad per crossed edge, closed and crack-free), then Laplacian smoothing and our own normals. Pure C#, tested headless. |
| `Scripts/FluidSurfaceRenderer.cs` | Rebuilds and draws one surface mesh per fluid phase every frame. |
| `Scripts/FluidParticleRenderer.cs` | Particle view: instanced spheres colored by phase, speed or density (16 color bins, one batch each). |
| `Scripts/FluidControlPanel.cs` | The control panel (IMGUI): sliders, buttons, history chart, stats. |
| `Scripts/FluidMaterials.cs` | Glossy water, transparent glass, checker floor. |
| `Scripts/FluidLab.cs` | Scene controller: builds the world and the display objects, samples the metrics, and handles the keys. |
| `Editor/FluidSceneCreator.cs` | The menu item that writes the scene. |
| `Tests/FluidScenarioTests.cs` | EditMode tests (Test Runner): every scenario stays finite and shows its physics. |
| `Tests/FluidSurfaceMesherTests.cs` | The surface of a 0.25 m³ block encloses 0.25 m³ (±10%), faces outward, and its top sits half a spacing above the particles. |

## Measured behavior and limits

Measured headless with the scenario settings, 2-3 sub-steps × 4 iterations:

- Density: after the dam break settles, `rho/rho0` averages 0.95 and peaks around 1.03. A free surface lowers the
  average.
- Rayleigh-Taylor: the mean height of the heavy phase drops from 1.05 m to 0.45 m in about 4 s, and the light
  phase rises from 0.35 m to 0.65 m.
- Buoyancy: the 0.2 cork floats about 50% submerged, against 20% for Archimedes. The 0.4 wood floats about 90%
  submerged, against 40%. In this solver, buoyancy comes from fluid-solid contacts, not from a pressure field,
  so floating bodies sit lower than in reality. Raising or lowering `solidDensityScale` from its default of 0.5
  makes it worse. This is the "buoyancy does not match real submersion depths" limitation of the paper (§11) and
  of the library README. The panel shows both values instead of hiding the gap.
- A light body resting on the floor of an empty basin only rises once water gets under it, so in the fountain the
  balls lift off a few seconds apart.
- Surface meshing costs 3-5 ms per frame headless for 1,000-1,500 particles, and more in the editor's Mono
  runtime. Switch to the particle view (V) if the frame rate drops.
