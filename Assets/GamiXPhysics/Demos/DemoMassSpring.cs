using GamiX.Physics.MassSpring;
using GamiX.Physics.Rendering;
using UnityEngine;

namespace GamiX.Physics.Demos
{
    /// <summary>
    /// Chapter 2 part 1: the same mass-spring-damper chain integrated with explicit Euler, semi-implicit Euler, Verlet
    /// and RK4, side by side. With the same time step, explicit Euler gains energy and explodes while the others stay
    /// stable (raise the stiffness to 400 and only RK4 survives). The chains bounce on the ground (chapter 4, app. 1).
    /// </summary>
    public sealed class DemoMassSpring : MonoBehaviour
    {
        public int masses = 10;
        public float stiffness = 200f;
        public float damping = 0.2f;
        public float timeStep = 1f / 60f;

        private MassSpringSystem[] systems;
        private Material[] materials;
        private readonly MassSpringIntegrator[] methods =
        {
            MassSpringIntegrator.ExplicitEuler, MassSpringIntegrator.SemiImplicitEuler,
            MassSpringIntegrator.Verlet, MassSpringIntegrator.RungeKutta4
        };
        private readonly Matrix4x4[] matrices = new Matrix4x4[64];

        private void Start()
        {
            DemoUtils.Visual(PrimitiveType.Plane, "Ground", Vector3.zero, new Vector3(2f, 1f, 1f), new Color(0.35f, 0.37f, 0.4f));
            systems = new MassSpringSystem[methods.Length];
            materials = new Material[methods.Length];
            for (int k = 0; k < methods.Length; k++)
            {
                var s = new MassSpringSystem { integrator = methods[k], restitution = 0.6f, friction = 0.2f };
                float x = -3f + 2f * k;
                int prev = s.AddParticle(new Vector3(x, 4f, 0f), 0f); // anchor
                for (int i = 1; i <= masses; i++)
                {
                    int p = s.AddParticle(new Vector3(x + 0.3f * i, 4f, 0f), 0.1f);
                    s.ForceObjects.Add(new SpringForce(prev, p, stiffness, damping, 0.3f));
                    prev = p;
                }
                s.ForceObjects.Add(new GravityForce(new Vector3(0f, -9.81f, 0f)));
                s.ForceObjects.Add(new ViscousDragForce(0.01f));
                s.Planes.Add(Geometry.Plane3D.FromPointNormal(Vector3.zero, Vector3.up));
                systems[k] = s;
                materials[k] = RenderUtils.CreateLitMaterial(Color.HSVToRGB(0.2f * k, 0.7f, 1f));
            }
        }

        private void Update()
        {
            Mesh mesh = RenderUtils.SphereMesh;
            for (int k = 0; k < systems.Length; k++)
            {
                MassSpringSystem s = systems[k];
                s.Step(timeStep);
                int n = Mathf.Min(s.Count, matrices.Length);
                for (int i = 0; i < n; i++)
                {
                    Vector3 p = s.Positions[i];
                    if (!VMath.IsFinite(p) || VMath.Length(p) > 1e4f) p = new Vector3(0f, -100f, 0f); // exploded
                    matrices[i] = RenderUtils.ScaleTranslation(p, 0.15f);
                }
                Graphics.RenderMeshInstanced(new RenderParams(materials[k]), mesh, 0, matrices, n);
            }
        }

        private void OnGUI()
        {
            if (systems == null) return;
            for (int k = 0; k < systems.Length; k++)
            {
                float e = systems[k].KineticEnergy();
                string value = float.IsNaN(e) || e > 1e8f ? "exploded" : e.ToString("F2");
                GUI.Label(new Rect(10, 10 + 20 * k, 600, 22), $"{methods[k]}: kinetic energy = {value}");
            }
        }
    }
}
