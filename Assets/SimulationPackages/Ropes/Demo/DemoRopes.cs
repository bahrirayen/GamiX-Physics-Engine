using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Demos;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Ropes
{
    /// <summary>
    /// Ropes coupled with rigid bodies (paper fig. 1): crates hanging from ropes pinned to a beam, and a rope tied
    /// between two crates. Grab and swing them with the left mouse button.
    /// </summary>
    public sealed class DemoRopes : MonoBehaviour
    {
        private void Awake()
        {
            UnifiedWorld world = DemoUtils.CreateUnifiedWorld();
            world.parameters.radius = 0.03f;
            world.parameters.substeps = 3;
            world.parameters.iterations = 4;
            DemoUtils.Ground(world);
            DemoUtils.Visual(PrimitiveType.Cube, "Beam", new Vector3(0f, 3f, 0f), new Vector3(4f, 0.1f, 0.1f), new Color(0.5f, 0.4f, 0.3f));

            var toActivate = new System.Collections.Generic.List<GameObject>();
            for (int k = 0; k < 3; k++)
            {
                float x = -1.2f + k * 1.2f;
                float length = 1.2f + 0.4f * k;
                ParticleRigidBody crate = DemoUtils.BodyWithMesh<ParticleRigidBody>(world, PrimitiveType.Cube, "Crate " + k,
                    new Vector3(x, 3f - length - 0.12f, 0f), new Vector3(0.24f, 0.24f, 0.24f), Color.HSVToRGB(0.08f * k + 0.05f, 0.7f, 0.9f));
                crate.particleMass = 0.5f;
                toActivate.Add(crate.gameObject);

                ParticleRope rope = DemoUtils.Body<ParticleRope>(world, "Rope " + k, Vector3.zero, new Color(0.9f, 0.85f, 0.7f));
                rope.startLocal = new Vector3(x, 2.95f, 0f);
                rope.endLocal = new Vector3(x, 3f - length + 0.03f, 0f);
                rope.pinStart = true;
                rope.attachEndTo = crate;
                toActivate.Add(rope.gameObject);
            }

            world.gameObject.SetActive(true);
            foreach (GameObject go in toActivate) go.SetActive(true);
        }
    }
}
