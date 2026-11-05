using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Demos;
using GamiX.Physics.Rendering;
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
            ApplyCheckerFloor();
            DemoUtils.Visual(PrimitiveType.Cube, "Beam", new Vector3(0f, 3f, 0f), new Vector3(4f, 0.1f, 0.1f), new Color(0.5f, 0.4f, 0.3f));

            var toActivate = new System.Collections.Generic.List<GameObject>();
            Color[] ballColors =
            {
                new Color(0.12f, 0.78f, 0.94f),
                new Color(0.98f, 0.68f, 0.12f),
                new Color(0.7f, 0.35f, 0.98f)
            };
            for (int k = 0; k < 3; k++)
            {
                float x = -1.2f + k * 1.2f;
                bool isMiddleCube = k == 1;
                float length = isMiddleCube ? 2.55f : 1.2f + 0.4f * k;
                float bodyDiameter = k == 0 ? 0.34f : isMiddleCube ? 0.36f : 0.32f;
                string bodyName = k == 0 ? "Turquoise Metal Ball" : isMiddleCube ? "Gold Metal Cube" : "Violet Metal Ball";
                PrimitiveType visualShape = isMiddleCube ? PrimitiveType.Cube : PrimitiveType.Sphere;
                ParticleRigidBody crate = DemoUtils.BodyWithMesh<ParticleRigidBody>(world, visualShape, bodyName,
                    new Vector3(x, 3f - length - 0.12f, 0f), new Vector3(bodyDiameter, bodyDiameter, bodyDiameter), Color.HSVToRGB(0.08f * k + 0.05f, 0.7f, 0.9f));
                crate.particleMass = 0.5f;
                crate.shape = isMiddleCube ? ParticleRigidShape.Box : ParticleRigidShape.Sphere;
                crate.size = Vector3.one;

                Material metal = RenderUtils.CreateLitMaterial(ballColors[k]);
                if (metal.HasProperty("_Metallic")) metal.SetFloat("_Metallic", 0.68f);
                if (metal.HasProperty("_Smoothness")) metal.SetFloat("_Smoothness", 0.84f);
                crate.GetComponent<MeshRenderer>().sharedMaterial = metal;
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

        private static void ApplyCheckerFloor()
        {
            GameObject floor = GameObject.Find("Ground");
            if (floor == null) return;

            const int textureSize = 128;
            const int squareSize = textureSize / 2;
            Texture2D checker = new Texture2D(textureSize, textureSize, TextureFormat.RGBA32, true)
            {
                name = "Ropes Checker Floor",
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Repeat
            };
            Color32 lightTile = new Color32(224, 228, 236, 255);
            Color32 darkTile = new Color32(176, 185, 198, 255);
            Color32[] pixels = new Color32[textureSize * textureSize];
            for (int y = 0; y < textureSize; y++)
            for (int x = 0; x < textureSize; x++)
            {
                bool light = ((x / squareSize) + (y / squareSize)) % 2 == 0;
                pixels[y * textureSize + x] = light ? lightTile : darkTile;
            }
            checker.SetPixels32(pixels);
            checker.Apply(true, true);

            Material grid = RenderUtils.CreateLitMaterial(Color.white);
            grid.name = "Ropes Checker Floor Material";
            if (grid.HasProperty("_BaseMap"))
            {
                grid.SetTexture("_BaseMap", checker);
                grid.SetTextureScale("_BaseMap", new Vector2(5f, 5f));
            }
            if (grid.HasProperty("_MainTex"))
            {
                grid.SetTexture("_MainTex", checker);
                grid.SetTextureScale("_MainTex", new Vector2(5f, 5f));
            }
            if (grid.HasProperty("_Metallic")) grid.SetFloat("_Metallic", 0f);
            if (grid.HasProperty("_Smoothness")) grid.SetFloat("_Smoothness", 0.12f);
            floor.GetComponent<MeshRenderer>().sharedMaterial = grid;
        }
    }
}
