using System.Collections.Generic;
using GamiX.Physics.Collision;
using GamiX.Physics.Geometry;
using GamiX.Physics.Integration;
using GamiX.Physics.MassSpring;
using GamiX.Physics.RigidBodies;
using GamiX.Physics.Unified;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Physics.Tests
{
    /// <summary>Integration (chapter 2), collisions (chapter 4), rigid bodies and the unified solver (paper).</summary>
    public class DynamicsTests
    {
        // ---------------------------------------------------------------- chapter 2 part 1

        [Test]
        public void Rk4_Slide14Example()
        {
            // dy/dt = -2 y t, y(0) = 1  =>  y(t) = exp(-t^2).
            float y = 1f, t = 0f, h = 0.1f;
            for (int i = 0; i < 10; i++) { y = ScalarOde.Rk4Step((yy, tt) => -2f * yy * tt, y, t, h); t += h; }
            Assert.AreEqual(Mathf.Exp(-1f), y, 1e-4f);
        }

        [Test]
        public void FreeFallWithDrag_Rk4BeatsEuler()
        {
            // dv/dt = g - v / tau (slide 6), compared with the analytic solution.
            float g = 9.81f, tau = 0.5f, dt = 0.1f;
            Vector3 xe = Vector3.zero, ve = Vector3.zero, xr = Vector3.zero, vr = Vector3.zero;
            AccelerationFunc a = (x, v, time) => new Vector3(0f, g, 0f) - v / tau;
            for (int i = 0; i < 20; i++)
            {
                PointIntegrators.ExplicitEuler(ref xe, ref ve, a, i * dt, dt);
                PointIntegrators.RungeKutta4(ref xr, ref vr, a, i * dt, dt);
            }
            float exact = PointIntegrators.FreeFallWithDragVelocity(0f, g, tau, 2f);
            Assert.Less(Mathf.Abs(vr.y - exact), 1e-3f);
            Assert.Less(Mathf.Abs(vr.y - exact), Mathf.Abs(ve.y - exact));
        }

        [Test]
        public void MassSpring_Rk4ConservesEnergyBetterThanEuler()
        {
            float EnergyAfter(MassSpringIntegrator method)
            {
                var s = new MassSpringSystem { integrator = method };
                int a = s.AddParticle(Vector3.zero, 0f);
                int b = s.AddParticle(new Vector3(1.5f, 0f, 0f), 1f);
                s.ForceObjects.Add(new SpringForce(a, b, 50f, 0f, 1f));
                for (int i = 0; i < 300; i++) s.Step(1f / 60f);
                Vector3 d = s.Positions[b] - s.Positions[a];
                float stretch = VMath.Length(d) - 1f;
                return s.KineticEnergy() + 0.5f * 50f * stretch * stretch;
            }
            float e0 = 0.5f * 50f * 0.25f;
            Assert.Less(Mathf.Abs(EnergyAfter(MassSpringIntegrator.RungeKutta4) - e0), 0.05f * e0);
            Assert.Greater(EnergyAfter(MassSpringIntegrator.ExplicitEuler), 2f * e0); // explicit Euler gains energy
        }

        // ---------------------------------------------------------------- chapter 4

        [Test]
        public void SweepAndPrune1D_Slide61()
        {
            // b3=1 b6=2 b1=3 e6=6 e1=7 b5=7.5 b2=7.7 e3=8 b4=8.9 e5=11 e4=11.2 e2=18 (interval i at index i-1).
            float[] b = { 3f, 7.7f, 1f, 8.9f, 7.5f, 2f };
            float[] e = { 7f, 18f, 8f, 11.2f, 11f, 6f };
            var pairs = new List<BodyPair>();
            SweepAndPrune1D.FindOverlaps(b, e, pairs);
            var found = new HashSet<(int, int)>();
            foreach (BodyPair p in pairs) found.Add((p.a + 1, p.b + 1));
            var expected = new[] { (1, 3), (1, 6), (2, 3), (2, 4), (2, 5), (3, 5), (3, 6), (4, 5) };
            Assert.AreEqual(expected.Length, found.Count);
            foreach (var pr in expected) Assert.IsTrue(found.Contains(pr), $"missing pair {pr}");
        }

        [Test]
        public void BroadPhases_FindTheSamePairs()
        {
            var rng = new System.Random(42);
            var boxes = new List<Aabb>();
            for (int i = 0; i < 300; i++)
            {
                Vector3 c = new Vector3((float)rng.NextDouble() * 10f, (float)rng.NextDouble() * 10f, (float)rng.NextDouble() * 10f);
                boxes.Add(Aabb.FromCenterExtents(c, Vector3.one * (0.2f + (float)rng.NextDouble() * 0.4f)));
            }
            HashSet<(int, int)> Run(IBroadPhase bp)
            {
                var pairs = new List<BodyPair>();
                bp.FindPairs(boxes, pairs);
                var set = new HashSet<(int, int)>();
                foreach (BodyPair p in pairs) Assert.IsTrue(set.Add((p.a, p.b)), "duplicate pair");
                return set;
            }
            var brute = Run(new BruteForceBroadPhase());
            var sap = new SweepAndPruneBroadPhase();
            var grid = new UniformGridBroadPhase();
            Assert.IsTrue(brute.SetEquals(Run(sap)));
            Assert.IsTrue(brute.SetEquals(Run(grid)));
            Assert.Less(sap.TestsLastFrame, 300 * 299 / 2);
            Assert.Less(grid.TestsLastFrame, 300 * 299 / 2);
        }

        [Test]
        public void ObbSat_Slide43()
        {
            Obb a = new Obb(Vector3.zero, Quat.Identity, Vector3.one * 0.5f);
            Quat yaw45 = Quat.FromAxisAngleDeg(Vector3.up, 45f);
            // The rotated box's corner (0.707 from its center) pierces the face x = 0.5.
            Assert.IsTrue(Overlap.ObbObb(a, new Obb(new Vector3(1.15f, 0f, 0f), yaw45, Vector3.one * 0.5f)));
            Assert.IsFalse(Overlap.ObbObb(a, new Obb(new Vector3(1.25f, 0f, 0f), yaw45, Vector3.one * 0.5f)));
            // Application 6: diagonal placement where the AABBs overlap but SAT finds the separating axis.
            Obb diagonal = new Obb(new Vector3(0.9f, 0f, 0.9f), yaw45, Vector3.one * 0.5f);
            Assert.IsTrue(a.ComputeAabb().Overlaps(diagonal.ComputeAabb()));
            Assert.IsFalse(Overlap.ObbObb(a, diagonal));
            var contacts = new List<Contact>();
            Assert.IsTrue(NarrowPhase.ObbObb(a, new Obb(new Vector3(0f, 0.95f, 0f), Quat.Identity, Vector3.one * 0.5f), contacts));
            Assert.AreEqual(4, contacts.Count); // face-face: the 4 corners of the incident face
            foreach (Contact c in contacts)
            {
                Assert.AreEqual(1f, c.normal.y, 1e-4f);
                Assert.AreEqual(0.05f, c.penetration, 1e-4f);
            }
        }

        [Test]
        public void TimeOfImpact_Application2()
        {
            // Projectile r = 5 cm at 40 m/s toward a wall face at x = 4.9 (wall 0.2 thick centered at 5): impact at x = 4.85.
            Plane3D face = Plane3D.FromPointNormal(new Vector3(4.9f, 0f, 0f), -Vector3.right);
            Vector3 p = new Vector3(4.5f, 0f, 0f);
            Assert.IsTrue(TimeOfImpact.SweptSpherePlane(p, 0.05f, new Vector3(40f, 0f, 0f), face, 1f / 60f, out float toi));
            Assert.AreEqual(4.85f, p.x + 40f * toi, 1e-4f);
            Assert.AreEqual(3, TimeOfImpact.SubSteps(40f, 1f / 60f, 0.2f, 0.05f));
        }

        // ---------------------------------------------------------------- chapter 2 part 2

        [Test]
        public void InertiaTensors_Slide33()
        {
            Mat3 box = InertiaTensor.Box(12f, new Vector3(1f, 2f, 3f));
            Assert.AreEqual(4f + 9f, box.m00, 1e-4f);
            Assert.AreEqual(1f + 9f, box.m11, 1e-4f);
            Assert.AreEqual(1f + 4f, box.m22, 1e-4f);
            Assert.AreEqual(0.4f * 2f * 9f, InertiaTensor.SolidSphere(2f, 3f).m00, 1e-4f);
            // Point masses filling a cube converge to the solid box formula.
            var pts = new List<Vector3>();
            var masses = new List<float>();
            int n = 20;
            for (int x = 0; x < n; x++) for (int y = 0; y < n; y++) for (int z = 0; z < n; z++)
            {
                pts.Add(new Vector3((x + 0.5f) / n - 0.5f, (y + 0.5f) / n - 0.5f, (z + 0.5f) / n - 0.5f));
                masses.Add(1f / (n * n * n));
            }
            Mat3 cloud = InertiaTensor.FromPointMasses(pts, masses, out Vector3 com, out float m);
            Assert.AreEqual(1f, m, 1e-4f);
            Assert.AreEqual(1f / 6f, cloud.m00, 1e-3f);
            Assert.AreEqual(0f, cloud.m01, 1e-5f);
        }

        [Test]
        public void RigidBody_TorqueFromOffCenterForce()
        {
            var b = new RigidBody(new BoxShape(Vector3.one * 0.5f), 1f, Vector3.zero, Quat.Identity);
            b.AddForceAtPoint(new Vector3(0f, 0f, 1f), new Vector3(1f, 0f, 0f));
            Assert.AreEqual(-1f, b.torque.y, 1e-5f); // tau = r x F = (1,0,0) x (0,0,1) = (0,-1,0)
        }

        [Test]
        public void RigidWorld_ElasticCollisionConservesMomentumAndEnergy()
        {
            var w = new RigidBodyWorld { gravity = Vector3.zero, allowSleeping = false };
            var a = w.AddBody(new RigidBody(new SphereShape(0.5f), 1f, new Vector3(-2f, 0f, 0f), Quat.Identity));
            var b = w.AddBody(new RigidBody(new SphereShape(0.5f), 3f, new Vector3(2f, 0.2f, 0f), Quat.Identity));
            foreach (RigidBody r in new[] { a, b }) { r.restitution = 1f; r.friction = 0f; r.linearDamping = 0f; r.angularDamping = 0f; }
            a.Velocity = new Vector3(4f, 0f, 0f);
            b.Velocity = new Vector3(-1f, 0f, 0f);
            Vector3 p0 = w.TotalLinearMomentum();
            float e0 = w.TotalKineticEnergy();
            for (int i = 0; i < 120; i++) w.Step(1f / 60f);
            Assert.Less(VMath.Distance(p0, w.TotalLinearMomentum()), 1e-3f);
            Assert.AreEqual(e0, w.TotalKineticEnergy(), 0.02f * e0);
            Assert.Less(a.Velocity.x, 3.9f); // they actually collided
        }

        [Test]
        public void RigidWorld_BoxStackStaysUp()
        {
            var w = new RigidBodyWorld();
            w.AddBody(RigidBody.CreateStatic(new PlaneShape(), Vector3.zero, Quat.Identity));
            var boxes = new List<RigidBody>();
            for (int i = 0; i < 5; i++)
                boxes.Add(w.AddBody(new RigidBody(new BoxShape(Vector3.one * 0.5f), 1f, new Vector3(0f, 0.5f + i, 0f), Quat.Identity)));
            for (int i = 0; i < 300; i++) w.Step(1f / 60f);
            for (int i = 0; i < 5; i++)
            {
                Assert.AreEqual(0.5f + i, boxes[i].position.y, 0.05f);
                Assert.Less(Mathf.Abs(boxes[i].position.x), 0.05f);
            }
        }

        // ---------------------------------------------------------------- unified particle solver (paper)

        [Test]
        public void Unified_FluidSettlesNearRestDensity()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f, multithreaded = false });
            s.AddShape(new KinematicPlane());
            s.AddShape(new KinematicBox { position = new Vector3(0f, 1f, 0f), halfExtents = new Vector3(0.4f, 1f, 0.4f), inverted = true });
            List<int> fluid = ParticleFactory.CreateFluidBlock(s, new Vector3(0f, 0.4f, 0f), new Vector3(0.75f, 0.75f, 0.75f));
            for (int i = 0; i < 180; i++) s.Step(1f / 60f);
            float avg = 0f;
            foreach (int i in fluid)
            {
                avg += s.densities[i];
                Assert.IsTrue(VMath.IsFinite(s.positions[i]));
                Assert.GreaterOrEqual(s.positions[i].y, 0f);
            }
            Assert.AreEqual(1f, avg / fluid.Count / s.RestDensity, 0.1f);
        }

        [Test]
        public void Unified_RigidShapeMatchingStaysRigid()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f, multithreaded = false });
            s.AddShape(new KinematicPlane());
            RigidGroup g = ParticleFactory.CreateRigidBox(s, new Vector3(0f, 1f, 0f), Quat.FromAxisAngleDeg(new Vector3(1, 1, 0), 30f),
                                                          new Vector3(0.4f, 0.3f, 0.3f));
            float d0 = VMath.Distance(s.positions[g.indices[0]], s.positions[g.indices[g.indices.Length - 1]]);
            for (int i = 0; i < 180; i++) s.Step(1f / 60f);
            float d1 = VMath.Distance(s.positions[g.indices[0]], s.positions[g.indices[g.indices.Length - 1]]);
            Assert.AreEqual(d0, d1, 0.01f);
            Assert.Less(g.center.y, 0.3f); // it fell and rests on the ground
        }

        [Test]
        public void Unified_FrictionMatchesCoulomb()
        {
            // A box sliding at 3 m/s with mu_k = 0.3 stops after v^2 / (2 mu g) = 1.53 m.
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f, staticFriction = 0.4f, dynamicFriction = 0.3f, multithreaded = false });
            s.AddShape(new KinematicPlane());
            RigidGroup g = ParticleFactory.CreateRigidBox(s, new Vector3(0f, 0.15f, 0f), Quat.Identity, new Vector3(0.4f, 0.3f, 0.3f), 1f, new Vector3(3f, 0f, 0f));
            for (int i = 0; i < 180; i++) s.Step(1f / 60f);
            Assert.AreEqual(9f / (2f * 0.3f * 9.81f), g.center.x, 0.08f);
        }

        [Test]
        public void Unified_DistanceConstraintsKeepAChainLength()
        {
            // A horizontal chain pinned at one end swings down; its links must keep their length.
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f, multithreaded = false });
            int n = 10;
            var chain = new int[n];
            int phase = Phase.Make(s.NewGroup(), 0);
            for (int k = 0; k < n; k++)
                chain[k] = s.AddParticle(new Vector3(0.1f * k, 2f, 0f), Vector3.zero, k == 0 ? 0f : 1f, phase);
            for (int k = 0; k + 1 < n; k++)
                s.AddDistanceConstraint(chain[k], chain[k + 1]);
            for (int i = 0; i < 120; i++) s.Step(1f / 60f);
            for (int k = 0; k + 1 < n; k++)
                Assert.AreEqual(0.1f, VMath.Distance(s.positions[chain[k]], s.positions[chain[k + 1]]), 0.01f);
            Assert.Less(s.positions[chain[n - 1]].y, 1.5f); // it did swing down
        }

        private sealed class CountingExtension : SolverExtension
        {
            public int forces, solves, velocities, ends;
            public override void ApplyForces(float h) => forces++;
            public override bool SolveConstraints(int iteration) { solves++; return false; }
            public override void UpdateVelocities(float h) => velocities++;
            public override void EndSubStep(float h) => ends++;
        }

        [Test]
        public void Unified_ExtensionHooksRunEverySubStep()
        {
            var p = new SolverParams { substeps = 2, iterations = 3, multithreaded = false };
            var s = new UnifiedSolver(p);
            s.AddParticle(Vector3.up, Vector3.zero, 1f, Phase.Make(s.NewGroup(), 0));
            CountingExtension e = s.GetOrAddExtension<CountingExtension>();
            Assert.AreSame(e, s.GetOrAddExtension<CountingExtension>());
            s.Step(1f / 60f);
            Assert.AreEqual(2, e.forces);
            Assert.AreEqual(2 * 3, e.solves);
            Assert.AreEqual(2, e.velocities);
            Assert.AreEqual(2, e.ends);
        }
    }
}
