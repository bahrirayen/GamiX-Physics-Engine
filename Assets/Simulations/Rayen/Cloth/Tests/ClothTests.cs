using GamiX.Physics;
using GamiX.Physics.Unified;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Sim.Cloth.Tests
{
    public class ClothTests
    {
        [Test]
        public void ClothStretchIsSmall()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.04f, multithreaded = false });
            ClothHandle c = ClothBuilder.CreateCloth(s, new Vector3(-0.5f, 2f, 0f), Vector3.right, Vector3.forward, 20, 20, 0.05f,
                                                     isPinned: (u, v) => v == 0);
            for (int i = 0; i < 120; i++) s.Step(1f / 60f);
            float maxRatio = 0f;
            for (int k = 0; k < s.DistanceConstraints.Count; k++)
            {
                DistanceConstraint d = s.DistanceConstraints[k];
                if (d.kind != 0) continue;
                maxRatio = Mathf.Max(maxRatio, VMath.Distance(s.positions[d.i], s.positions[d.j]) / d.restLength);
            }
            Assert.Less(maxRatio, 1.1f);
        }

        [Test]
        public void TethersLimitTheDistanceToTheAnchor()
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.04f, multithreaded = false });
            ClothHandle c = ClothBuilder.CreateCloth(s, new Vector3(-0.5f, 2f, 0f), Vector3.right, Vector3.forward, 12, 12, 0.05f,
                                                     stretchStiffness: 0.2f, isPinned: (u, v) => v == 0);
            ClothBuilder.AddClothTethers(s, c, 1.05f);
            ClothTethers tethers = s.GetExtension<ClothTethers>();
            Assert.IsNotNull(tethers);
            for (int i = 0; i < 120; i++) s.Step(1f / 60f);
            for (int k = 0; k < tethers.Tethers.Count; k++)
            {
                TetherConstraint t = tethers.Tethers[k];
                Assert.LessOrEqual(VMath.Distance(s.positions[t.particle], s.positions[t.anchor]), t.maxLength * 1.02f);
            }
        }

        private static (UnifiedSolver, ClothHandle, ClothTearing) FlatCloth(int res)
        {
            var s = new UnifiedSolver(new SolverParams { radius = 0.04f, multithreaded = false, gravity = Vector3.zero });
            int first = s.DistanceConstraints.Count;
            // Spacing 1/(res-1) on the XZ plane at y = 1.
            ClothHandle c = ClothBuilder.CreateCloth(s, new Vector3(0f, 1f, 0f), Vector3.right, Vector3.forward, res, res, 0.05f);
            var tearing = new ClothTearing(s, c, first, s.DistanceConstraints.Count - first);
            return (s, c, tearing);
        }

        [Test]
        public void CutAcrossSplitsTheClothInTwoPieces()
        {
            var (s, c, tearing) = FlatCloth(11);
            Assert.AreEqual(1, tearing.CountPieces());
            // Blade along column 5 (x = 0.5, spacing 0.1), from one border to the other.
            int cut = tearing.Cut(new Vector3(0.5f, 1f, 0f), new Vector3(0.5f, 1f, 1f), 0.05f);
            Assert.AreEqual(10, cut);
            Assert.IsTrue(tearing.Update());
            Assert.AreEqual(2, tearing.CountPieces());
            Assert.AreEqual(11, tearing.SplitCount); // every vertex of the column is split in two
            Assert.AreEqual(11 * 11 + tearing.SplitCount, s.Count);

            // No intact constraint may still join the two sides of the cut.
            for (int k = 0; k < s.DistanceConstraints.Count; k++)
            {
                DistanceConstraint d = s.DistanceConstraints[k];
                if (d.broken) continue;
                Assert.AreEqual(Side(s, tearing, d.i), Side(s, tearing, d.j), $"constraint {k} crosses the cut");
            }
        }

        // Side of the cut of a particle, from the rest position of the triangles that use it.
        private static bool Side(UnifiedSolver s, ClothTearing tearing, int particle)
        {
            var tris = new System.Collections.Generic.List<int>();
            tearing.GetMeshTriangles(tris);
            Vector3 sum = Vector3.zero;
            int n = 0;
            for (int t = 0; t < tris.Count; t += 3)
            {
                bool uses = false;
                for (int k = 0; k < 3; k++) uses |= tearing.MeshParticle(tris[t + k]) == particle;
                if (!uses) continue;
                for (int k = 0; k < 3; k++) sum += s.restPositions[tearing.MeshParticle(tris[t + k])];
                n += 3;
            }
            return sum.x / n < 0.5f;
        }

        [Test]
        public void PartialCutKeepsOnePiece()
        {
            var (s, c, tearing) = FlatCloth(11);
            // A slit that stops in the middle of the cloth opens a hole but does not separate it.
            tearing.Cut(new Vector3(0.5f, 1f, 0f), new Vector3(0.5f, 1f, 0.5f), 0.05f);
            tearing.Update();
            Assert.AreEqual(1, tearing.CountPieces());
            Assert.Greater(tearing.SplitCount, 0);
        }

        [Test]
        public void TornPieceFallsAndTheRestStaysUp()
        {
            // A cloth pinned along its top edge, cut across: the bottom part falls, the top part stays.
            var s = new UnifiedSolver(new SolverParams { radius = 0.04f, multithreaded = false });
            int first = s.DistanceConstraints.Count;
            ClothHandle c = ClothBuilder.CreateCloth(s, new Vector3(0f, 2f, 0f), Vector3.right, Vector3.up, 11, 11, 0.05f,
                                                     isPinned: (u, v) => v == 10);
            var tearing = new ClothTearing(s, c, first, s.DistanceConstraints.Count - first);
            tearing.Cut(new Vector3(0f, 2.5f, 0f), new Vector3(1f, 2.5f, 0f), 0.05f);
            tearing.Update();
            Assert.AreEqual(2, tearing.CountPieces());
            for (int i = 0; i < 90; i++) s.Step(1f / 60f);
            float lowest = float.MaxValue, highestFree = float.MinValue;
            for (int i = 0; i < s.Count; i++)
            {
                lowest = Mathf.Min(lowest, s.positions[i].y);
                if (s.invMass[i] > 0f && s.restPositions[i].y > 2.55f) highestFree = Mathf.Max(highestFree, s.positions[i].y);
            }
            Assert.Less(lowest, 1.2f, "the cut piece should have fallen");
            Assert.Greater(highestFree, 2.3f, "the pinned part should still hang");
        }
    }
}
