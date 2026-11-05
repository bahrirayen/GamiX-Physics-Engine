using GamiX.Physics;
using GamiX.Physics.Unified;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Sim.Yassine.Tests
{
    public class FluidSurfaceMesherTests
    {
        [Test]
        public void BlockSurface_EnclosesTheBlockVolumeAndFacesOutward()
        {
            // 10 x 5 x 5 particles on a 0.1 m lattice stand for 0.25 m^3 of water.
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f });
            var idx = ParticleFactory.CreateFluidBlock(s, new Vector3(0f, 0.5f, 0f), new Vector3(1f, 0.5f, 0.5f), jitter: 0f);
            var m = new FluidSurfaceMesher();
            m.Build(s, idx);

            Assert.Greater(m.triangles.Count, 0);
            // Positive volume means Cross(b - a, c - a) points outward (Unity front faces). Within 10 %.
            Assert.AreEqual(0.25f, m.EnclosedVolume(), 0.025f);

            // The top face sits half a spacing above the highest particle centers (y = 0.7).
            float top = float.MinValue;
            foreach (Vector3 v in m.vertices) top = Mathf.Max(top, v.y);
            Assert.AreEqual(0.75f, top, 0.02f);

            foreach (Vector3 n in m.normals)
                Assert.AreEqual(1f, VMath.Length(n), 1e-3f);
        }

        [Test]
        public void TwoSeparateDrops_GiveTwoClosedPieces()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.05f });
            int phase = Phase.Make(s.NewGroup(), Phase.Fluid);
            int a = s.AddParticle(new Vector3(0f, 1f, 0f), Vector3.zero, 1f, phase);
            int b = s.AddParticle(new Vector3(1f, 1f, 0f), Vector3.zero, 1f, phase);
            var m = new FluidSurfaceMesher();
            m.Build(s, new[] { a, b });
            Assert.Greater(m.EnclosedVolume(), 0f);
            float left = 0f, right = 0f;
            foreach (Vector3 v in m.vertices)
            {
                if (v.x < 0.5f) left++;
                else right++;
            }
            Assert.AreEqual(left, right, "both drops must be meshed the same way");
        }

        [Test]
        public void NoParticles_GiveAnEmptyMesh()
        {
            var s = new UnifiedSolver(new SolverParams());
            var m = new FluidSurfaceMesher();
            m.Build(s, new int[0]);
            Assert.AreEqual(0, m.vertices.Count);
            Assert.AreEqual(0, m.triangles.Count);
        }
    }
}
