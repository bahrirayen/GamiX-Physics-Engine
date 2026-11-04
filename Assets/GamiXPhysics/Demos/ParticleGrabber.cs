using GamiX.Physics.Components;
using GamiX.Physics.Geometry;
using GamiX.Physics.Unified;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Physics.Demos
{
    /// <summary>
    /// Left mouse: grab a particle (ray cast against particles, chapter 3) and drag it. The grabbed particle is
    /// temporarily pinned and moved, so the constraints drag the rest of the object (cloth, rope, rigid, balloon).
    /// Space: radial impulse under the mouse. P: pause.
    /// </summary>
    public sealed class ParticleGrabber : MonoBehaviour
    {
        public UnifiedWorld world;
        public float impulseRadius = 0.6f;
        public float impulseSpeed = 6f;

        private int grabbed = -1;
        private float grabbedInvMass;
        private float grabDistance;
        private Camera cam;

        private void Start()
        {
            cam = Camera.main;
            if (world == null) world = FindAnyObjectByType<UnifiedWorld>();
            if (world != null) world.BeforeStep += OnBeforeStep;
        }

        private void OnDestroy()
        {
            if (world != null) world.BeforeStep -= OnBeforeStep;
        }

        private Ray3D MouseRay()
        {
            Vector2 mp = Mouse.current.position.ReadValue();
            UnityEngine.Ray r = cam.ScreenPointToRay(new Vector3(mp.x, mp.y, 0f));
            return new Ray3D(r.origin, r.direction);
        }

        private void Update()
        {
            if (world == null || world.Solver == null || cam == null || Mouse.current == null) return;
            UnifiedSolver solver = world.Solver;
            Keyboard kb = Keyboard.current;
            if (kb != null && kb.pKey.wasPressedThisFrame) world.paused = !world.paused;

            if (Mouse.current.leftButton.wasPressedThisFrame && solver.RaycastParticles(MouseRay(), 100f, out int p, out float t))
            {
                grabbed = p;
                grabbedInvMass = solver.invMass[p];
                grabDistance = t;
                solver.SetInverseMass(p, 0f);
            }
            if (Mouse.current.leftButton.wasReleasedThisFrame && grabbed >= 0)
            {
                solver.SetInverseMass(grabbed, grabbedInvMass);
                grabbed = -1;
            }
            if (kb != null && kb.spaceKey.wasPressedThisFrame)
            {
                Ray3D ray = MouseRay();
                Vector3 center = solver.RaycastParticles(ray, 100f, out _, out float hitT) ? ray.GetPoint(hitT) : ray.GetPoint(5f);
                solver.AddRadialImpulse(center, impulseRadius, impulseSpeed);
            }
        }

        private void OnBeforeStep(UnifiedSolver solver, float h)
        {
            if (grabbed < 0) return;
            solver.MovePinnedParticle(grabbed, MouseRay().GetPoint(grabDistance));
        }
    }
}
