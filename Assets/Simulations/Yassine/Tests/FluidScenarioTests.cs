using System.Linq;
using GamiX.Physics.Unified;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Sim.Yassine.Tests
{
    /// <summary>
    /// Headless checks of the fluid scenarios (no scene, no MonoBehaviour): each one runs a few simulated seconds and
    /// must stay finite and show the physics it is built for. Thresholds have a margin over the values measured
    /// when the scenarios were tuned.
    /// </summary>
    public class FluidScenarioTests
    {
        private static ScenarioSetup Run(FluidScenario scenario, float seconds, out UnifiedSolver solver)
        {
            solver = new UnifiedSolver(FluidScenarios.Parameters(scenario));
            ScenarioSetup setup = FluidScenarios.Build(solver, scenario);
            int frames = Mathf.RoundToInt(seconds * 60f);
            for (int f = 0; f < frames; f++)
                solver.Step(1f / 60f);
            Assert.IsTrue(FluidStats.AllFinite(solver), "positions and velocities must stay finite");
            return setup;
        }

        [Test]
        public void DamBreak_SpreadsAcrossTheTankAndStaysInside()
        {
            ScenarioSetup setup = Run(FluidScenario.DamBreak, 2f, out UnifiedSolver s);
            KinematicBox tank = setup.tanks[0];
            float r = s.parameters.radius;
            foreach (int i in setup.AllFluid)
            {
                Vector3 p = s.positions[i] - tank.position;
                Assert.LessOrEqual(Mathf.Abs(p.x), tank.halfExtents.x + r, "fluid left the tank");
                Assert.LessOrEqual(Mathf.Abs(p.z), tank.halfExtents.z + r, "fluid left the tank");
                Assert.GreaterOrEqual(s.positions[i].y, -r, "fluid went through the floor");
            }
            // The column started against the left wall (x < -0.6) and must reach the right wall.
            Assert.Greater(FluidStats.FrontX(s, setup.AllFluid), 1.3f);
            // Water level drops from 1.2 m to about 0.4 m once spread out.
            Assert.Less(FluidStats.MeanHeight(s, setup.AllFluid), 0.3f);
        }

        [Test]
        public void DamBreak_KeepsTheRestDensity()
        {
            ScenarioSetup setup = Run(FluidScenario.DamBreak, 3f, out UnifiedSolver s);
            FluidStats.DensityRatio(s, setup.AllFluid, out float avg, out float max);
            Assert.Less(max, 1.15f, "the unilateral density constraint (eq. 26) must prevent compression");
            Assert.Greater(avg, 0.85f);
        }

        [Test]
        public void Buoyancy_LightCratesFloatAndTheHeavyOneSinks()
        {
            ScenarioSetup setup = Run(FluidScenario.Buoyancy, 5f, out UnifiedSolver s);
            float surface = FluidStats.SurfaceHeight(s, setup.AllFluid);
            FloatingBody cork = setup.bodies.First(b => b.relativeDensity < 0.3f);
            FloatingBody wood = setup.bodies.First(b => b.relativeDensity > 0.3f && b.relativeDensity < 1f);
            FloatingBody steel = setup.bodies.First(b => b.relativeDensity > 1f);

            Assert.Less(FluidStats.Bottom(s, steel.rigid), 0.05f, "a crate denser than water must sink to the floor");
            Assert.Greater(FluidStats.Bottom(s, cork.rigid), 0.15f, "the cork must float");
            Assert.Greater(FluidStats.Bottom(s, wood.rigid), 0.1f, "the wood must float");

            float corkSub = FluidStats.SubmergedFraction(s, cork.rigid, surface);
            float woodSub = FluidStats.SubmergedFraction(s, wood.rigid, surface);
            Assert.Less(corkSub, 0.7f, "the cork must stick out of the water");
            Assert.Less(corkSub, woodSub, "the lighter crate must float higher (Archimedes)");
        }

        [Test]
        public void RayleighTaylor_HeavyFluidEndsBelowTheLightOne()
        {
            ScenarioSetup setup = Run(FluidScenario.RayleighTaylor, 4f, out UnifiedSolver s);
            FluidBlock light = setup.fluids.First(b => b.particleMass < 2f);
            FluidBlock heavy = setup.fluids.First(b => b.particleMass > 2f);
            float yLight = FluidStats.MeanHeight(s, light.particles);
            float yHeavy = FluidStats.MeanHeight(s, heavy.particles);
            // At the start the heavy layer is 0.7 m above the light one.
            Assert.Less(yHeavy, yLight - 0.1f, $"heavy {yHeavy:F2} m, light {yLight:F2} m");
        }

        [Test]
        public void Fountain_FillsTheBasinAndTheOutletBoundsTheParticleCount()
        {
            ScenarioSetup setup = Run(FluidScenario.Fountain, 14f, out UnifiedSolver s);
            FluidEmitter e = setup.emitter;
            Assert.Greater(e.TotalDrained, 200, "the overflow outlet must remove water once the basin is full");
            Assert.LessOrEqual(e.Alive, e.maxParticles);
            Assert.AreEqual(e.TotalEmitted - e.TotalDrained, e.Alive);
            Assert.AreEqual(e.Alive, s.FluidCount);
            Assert.Greater(FluidStats.MeanHeight(s, e.Particles), 0.15f);
            // The light balls are lifted by the rising water.
            foreach (FloatingBody b in setup.bodies)
                Assert.Greater(FluidStats.Bottom(s, b.rigid), 0.2f, $"{b.name} should float");
        }

        [Test]
        public void Emitter_EmitsOnTheLatticeAndStopsAtTheLimit()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f, multithreaded = false });
            s.AddShape(new KinematicPlane());
            FluidEmitter e = s.AddExtension(new FluidEmitter
            {
                nozzle = new Vector3(0f, 1f, 0f), direction = Vector3.down, speed = 2f, nozzleRadius = 0.1f, maxParticles = 100
            });
            for (int f = 0; f < 60; f++)
                s.Step(1f / 60f);
            Assert.AreEqual(0, e.TotalEmitted % 5, "a disc of radius 0.1 on a 0.1 lattice has 5 points per layer");
            Assert.LessOrEqual(e.Alive, 100);
            Assert.Greater(e.Alive, 90, "2 m/s over 1 s is 20 layers of 5 particles, capped at 100");
            Assert.IsTrue(FluidStats.AllFinite(s));
        }
    }
}
