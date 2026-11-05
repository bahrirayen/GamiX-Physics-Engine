using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    public enum FluidScenario
    {
        /// <summary>Column of water released in a tank, hits an obstacle and pushes a floating crate.</summary>
        DamBreak,
        /// <summary>Crates lighter and heavier than water dropped in a pool (two-way coupling, section 7.1.1).</summary>
        Buoyancy,
        /// <summary>Heavy fluid on top of a light one: the interface becomes unstable and they swap (multi-phase).</summary>
        RayleighTaylor,
        /// <summary>A jet fills a basin with floating balls; an overflow outlet removes the excess (emitter extension).</summary>
        Fountain
    }

    /// <summary>A block of fluid created by a scenario.</summary>
    public sealed class FluidBlock
    {
        public string name;
        public Color color;
        public int group;
        public float particleMass;
        public List<int> particles;
    }

    /// <summary>A rigid body (shape matching, section 5) floating or sinking in a scenario.</summary>
    public sealed class FloatingBody
    {
        public string name;
        public Color color;
        public bool sphere;
        /// <summary>Box: full size. Sphere: x = diameter.</summary>
        public Vector3 size;
        /// <summary>Density relative to the fluid (particle mass ratio: both use the same lattice spacing).</summary>
        public float relativeDensity;
        public RigidGroup rigid;
    }

    /// <summary>Everything a scenario created in the solver, for display, readouts and tests.</summary>
    public sealed class ScenarioSetup
    {
        public FluidScenario scenario;
        public string title;
        public string description;
        /// <summary>Containers (inverted boxes).</summary>
        public readonly List<KinematicBox> tanks = new List<KinematicBox>();
        /// <summary>Solid obstacles.</summary>
        public readonly List<KinematicBox> obstacles = new List<KinematicBox>();
        public readonly List<FluidBlock> fluids = new List<FluidBlock>();
        public readonly List<FloatingBody> bodies = new List<FloatingBody>();
        public FluidEmitter emitter;

        /// <summary>Indices of every fluid particle alive (blocks and emitter).</summary>
        public IEnumerable<int> AllFluid
        {
            get
            {
                foreach (FluidBlock b in fluids)
                    foreach (int i in b.particles)
                        yield return i;
                if (emitter != null)
                    foreach (int i in emitter.Particles)
                        yield return i;
            }
        }
    }

    /// <summary>
    /// Builds the fluid scenarios in a <see cref="UnifiedSolver"/>. Pure physics: no MonoBehaviour, so the same code
    /// runs in the scene (<see cref="FluidLab"/>) and headless in the tests.
    /// </summary>
    public static class FluidScenarios
    {
        public static readonly Color Water = new Color(0.15f, 0.45f, 0.95f);
        public static readonly Color HeavyFluid = new Color(0.9f, 0.35f, 0.1f);
        public static readonly Color LightFluid = new Color(0.2f, 0.75f, 0.95f);

        /// <summary>Solver parameters tuned for each scenario.</summary>
        public static SolverParams Parameters(FluidScenario scenario)
        {
            var p = new SolverParams
            {
                radius = 0.05f,
                substeps = 2,
                iterations = 4,
                viscosity = 0.01f,
                cohesion = 0.05f,
                vorticityConfinement = 0f,
                fluidFriction = 0f,
                staticFriction = 0.4f,
                dynamicFriction = 0.3f,
            };
            switch (scenario)
            {
                case FluidScenario.DamBreak:
                    p.substeps = 3;
                    p.vorticityConfinement = 1f;
                    break;
                case FluidScenario.Buoyancy:
                    p.viscosity = 0.05f; // calmer surface so the crates settle at their floating depth
                    break;
                case FluidScenario.RayleighTaylor:
                    p.cohesion = 0.1f; // cohesion acts within a phase only: it keeps the interface sharp
                    p.viscosity = 0.02f;
                    break;
                case FluidScenario.Fountain:
                    p.substeps = 3;
                    break;
            }
            return p;
        }

        public static ScenarioSetup Build(UnifiedSolver solver, FluidScenario scenario)
        {
            var setup = new ScenarioSetup { scenario = scenario };
            solver.AddShape(new KinematicPlane());
            switch (scenario)
            {
                case FluidScenario.DamBreak: DamBreak(solver, setup); break;
                case FluidScenario.Buoyancy: Buoyancy(solver, setup); break;
                case FluidScenario.RayleighTaylor: RayleighTaylor(solver, setup); break;
                default: Fountain(solver, setup); break;
            }
            return setup;
        }

        // ---------------------------------------------------------------- scenarios

        private static void DamBreak(UnifiedSolver s, ScenarioSetup setup)
        {
            setup.title = "Dam break";
            setup.description = "A water column collapses (density constraint, eq. 26), splashes over an obstacle " +
                                "and pushes a floating crate (two-way coupling).";
            Vector3 tankSize = new Vector3(3.2f, 1.6f, 0.8f);
            Tank(s, setup, new Vector3(0f, 0.5f * tankSize.y, 0f), tankSize);
            Obstacle(s, setup, new Vector3(0.5f, 0.15f, 0f), new Vector3(0.16f, 0.3f, tankSize.z));
            Fluid(s, setup, "Water", Water, new Vector3(-1.6f + 0.5f, 0.6f, 0f), new Vector3(1.0f, 1.2f, 0.8f), 1f);
            Body(s, setup, "Crate", new Color(0.75f, 0.55f, 0.3f), false, new Vector3(1.1f, 0.15f, 0f), new Vector3(0.3f, 0.3f, 0.3f), 0.4f);
        }

        private static void Buoyancy(UnifiedSolver s, ScenarioSetup setup)
        {
            setup.title = "Buoyancy";
            setup.description = "Crates at 0.2x, 0.4x and 2x the density of water. Archimedes: a floating body is " +
                                "submerged by rho_body / rho_fluid; a heavier one sinks.";
            Vector3 tankSize = new Vector3(2.4f, 1.4f, 0.8f);
            Tank(s, setup, new Vector3(0f, 0.5f * tankSize.y, 0f), tankSize);
            Fluid(s, setup, "Water", Water, new Vector3(0f, 0.35f, 0f), new Vector3(2.4f, 0.7f, 0.8f), 1f);
            var size = new Vector3(0.4f, 0.4f, 0.4f);
            Body(s, setup, "Cork (0.2)", new Color(0.95f, 0.85f, 0.5f), false, new Vector3(-0.75f, 1.05f, 0f), size, 0.2f);
            Body(s, setup, "Wood (0.4)", new Color(0.7f, 0.45f, 0.2f), false, new Vector3(0f, 1.05f, 0f), size, 0.4f);
            Body(s, setup, "Steel (2.0)", new Color(0.55f, 0.58f, 0.62f), false, new Vector3(0.75f, 1.05f, 0f), size, 2f);
        }

        private static void RayleighTaylor(UnifiedSolver s, ScenarioSetup setup)
        {
            setup.title = "Rayleigh-Taylor";
            setup.description = "A fluid 3x heavier rests on a light one. Mass-weighted density corrections " +
                                "(section 7.1.1) make the heavy phase sink in fingers and the light one rise.";
            Vector3 tankSize = new Vector3(1.6f, 1.6f, 0.4f);
            Tank(s, setup, new Vector3(0f, 0.5f * tankSize.y, 0f), tankSize);
            Fluid(s, setup, "Light fluid", LightFluid, new Vector3(0f, 0.35f, 0f), new Vector3(1.6f, 0.7f, 0.4f), 1f);
            Fluid(s, setup, "Heavy fluid", HeavyFluid, new Vector3(0f, 1.05f, 0f), new Vector3(1.6f, 0.7f, 0.4f), 3f);
        }

        private static void Fountain(UnifiedSolver s, ScenarioSetup setup)
        {
            setup.title = "Fountain";
            setup.description = "A nozzle fills an empty basin and the balls rise with the water. The overflow outlet " +
                                "high on the right wall removes the excess, so the level settles near 0.5 m (FluidEmitter extension).";
            Vector3 tankSize = new Vector3(2.4f, 1.0f, 0.8f);
            Tank(s, setup, new Vector3(0f, 0.5f * tankSize.y, 0f), tankSize);
            Body(s, setup, "Ball", new Color(0.95f, 0.3f, 0.3f), true, new Vector3(0.25f, 0.25f, -0.12f), new Vector3(0.5f, 0f, 0f), 0.15f);
            Body(s, setup, "Ball", new Color(0.3f, 0.85f, 0.4f), true, new Vector3(0.75f, 0.25f, 0.12f), new Vector3(0.5f, 0f, 0f), 0.15f);

            FluidEmitter e = s.AddExtension(new FluidEmitter
            {
                nozzle = new Vector3(-1.05f, 1.4f, 0f),
                direction = new Vector3(1f, -0.25f, 0f),
                speed = 2.2f,
                nozzleRadius = 0.16f,
                maxParticles = 1600,
                drainCenter = new Vector3(1.1f, 0.8f, 0f),
                drainSize = new Vector3(0.2f, 0.6f, 0.8f)
            });
            setup.emitter = e;
        }

        // ---------------------------------------------------------------- helpers

        private static void Tank(UnifiedSolver s, ScenarioSetup setup, Vector3 center, Vector3 size)
        {
            var box = new KinematicBox { position = center, halfExtents = size * 0.5f, inverted = true };
            s.AddShape(box);
            setup.tanks.Add(box);
        }

        private static void Obstacle(UnifiedSolver s, ScenarioSetup setup, Vector3 center, Vector3 size)
        {
            var box = new KinematicBox { position = center, halfExtents = size * 0.5f };
            s.AddShape(box);
            setup.obstacles.Add(box);
        }

        private static void Fluid(UnifiedSolver s, ScenarioSetup setup, string name, Color color, Vector3 center, Vector3 size,
                                  float particleMass)
        {
            int group = s.NewGroup();
            List<int> particles = ParticleFactory.CreateFluidBlock(s, center, size, particleMass, default, group);
            setup.fluids.Add(new FluidBlock { name = name, color = color, group = group, particleMass = particleMass, particles = particles });
        }

        /// <summary>
        /// Rigid body whose particles weigh relativeDensity times a unit-mass fluid particle. Rigid and fluid particles
        /// sit on lattices of the same spacing, so this is also the density ratio.
        /// </summary>
        private static void Body(UnifiedSolver s, ScenarioSetup setup, string name, Color color, bool sphere, Vector3 position,
                                 Vector3 size, float relativeDensity)
        {
            RigidGroup g = sphere
                ? ParticleFactory.CreateRigidSphere(s, position, 0.5f * size.x, relativeDensity)
                : ParticleFactory.CreateRigidBox(s, position, Quat.Identity, size, relativeDensity);
            setup.bodies.Add(new FloatingBody
            {
                name = name, color = color, sphere = sphere, size = size, relativeDensity = relativeDensity, rigid = g
            });
        }
    }
}
