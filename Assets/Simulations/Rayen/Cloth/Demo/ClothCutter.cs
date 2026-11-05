using GamiX.Physics.Components;
using GamiX.Physics.Geometry;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Blade tool: hold C and move the mouse over a cloth. The blade path between two frames cuts every constraint
    /// it passes through; <see cref="ClothTearing"/> then splits the mesh along the cut.
    /// </summary>
    public sealed class ClothCutter : MonoBehaviour
    {
        public UnifiedWorld world;
        public bool Cutting { get; private set; }
        public int CutCount { get; private set; }

        private Camera cam;
        private bool hasLast;
        private Vector3 last;

        private void Start()
        {
            cam = Camera.main;
            if (world == null) world = FindAnyObjectByType<UnifiedWorld>();
        }

        private void Update()
        {
            Keyboard kb = Keyboard.current;
            // The key labeled C in the current layout (AZERTY or QWERTY).
            var cKey = kb != null ? kb.FindKeyOnCurrentKeyboardLayout("c") : null;
            Cutting = cKey != null && cKey.isPressed && Mouse.current != null;
            if (!Cutting || world == null || world.Solver == null || cam == null)
            {
                hasLast = false;
                return;
            }
            Vector2 mp = Mouse.current.position.ReadValue();
            UnityEngine.Ray r = cam.ScreenPointToRay(new Vector3(mp.x, mp.y, 0f));
            var ray = new Ray3D(r.origin, r.direction);
            if (!world.Solver.RaycastParticles(ray, 100f, out int particle, out _))
            {
                hasLast = false;
                return;
            }
            // Follow the cloth itself (particle centers), not the front of the particle spheres.
            Vector3 hit = world.Solver.positions[particle];
            if (hasLast)
            {
                foreach (ParticleCloth cloth in FindObjectsByType<ParticleCloth>())
                    if (cloth.cuttable)
                        CutCount += cloth.Cut(last, hit);
            }
            last = hit;
            hasLast = true;
        }
    }
}
