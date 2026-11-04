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
    }
}
