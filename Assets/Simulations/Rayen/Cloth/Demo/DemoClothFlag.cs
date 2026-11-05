using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Demos;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Cloth (paper section 8): a flag in the wind (aerodynamic drag/lift per triangle, tethers) and a sheet falling
    /// on a sphere. Drag particles with the left mouse button.
    /// </summary>
    public sealed class DemoClothFlag : MonoBehaviour
    {
        public Vector3 wind = new Vector3(8f, 0f, 2f);

        private void Awake()
        {
            UnifiedWorld world = DemoUtils.CreateUnifiedWorld();
            world.parameters.radius = 0.03f;
            world.parameters.substeps = 3;
            world.parameters.iterations = 4;
            world.parameters.wind = wind;
            ClothAerodynamics aero = world.gameObject.AddComponent<ClothAerodynamics>();
            aero.drag = 1.5f;
            aero.lift = 0.8f;
            DemoUtils.Ground(world);

            DemoUtils.Visual(PrimitiveType.Cylinder, "Pole", new Vector3(-1.5f, 1.5f, 0f), new Vector3(0.06f, 1.5f, 0.06f), new Color(0.5f, 0.4f, 0.3f));

            ParticleCloth flag = DemoUtils.Body<ParticleCloth>(world, "Flag", new Vector3(-0.75f, 2.4f, 0f), new Color(0.85f, 0.15f, 0.2f));
            flag.transform.rotation = Quaternion.Euler(90f, 0f, 0f); // cloth plane XZ -> vertical XY
            flag.dimensions = new Vector2(1.5f, 1f);
            flag.resolution = new Vector2Int(26, 18);
            flag.pinMode = ClothPinMode.LeftEdge;
            flag.particleMass = 0.005f; // ~2 kg flag: light enough for the wind to lift it

            DemoUtils.Collider(world, "Sphere Collider", ColliderShape.Sphere, new Vector3(2f, 0.6f, 0f), new Vector3(0.5f, 0.5f, 0.5f));
            DemoUtils.Visual(PrimitiveType.Sphere, "Sphere", new Vector3(2f, 0.6f, 0f), Vector3.one * 0.98f, new Color(0.3f, 0.7f, 0.4f));
            ParticleCloth sheet = DemoUtils.Body<ParticleCloth>(world, "Sheet", new Vector3(2f, 1.6f, 0f), new Color(0.9f, 0.9f, 0.95f));
            sheet.dimensions = new Vector2(1.6f, 1.6f);
            sheet.resolution = new Vector2Int(28, 28);
            sheet.pinMode = ClothPinMode.None;
            sheet.useTethers = false;
            sheet.aerodynamics = false;

            world.gameObject.SetActive(true);
            flag.gameObject.SetActive(true);
            sheet.gameObject.SetActive(true);
        }
    }
}
