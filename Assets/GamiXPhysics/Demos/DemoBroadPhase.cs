using System.Collections.Generic;
using GamiX.Physics.Collision;
using GamiX.Physics.Geometry;
using GamiX.Physics.Rendering;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Physics.Demos
{
    /// <summary>
    /// Chapter 4, applications 7-8 ("gaz parfait dans une boite"): N spheres with elastic collisions in a box.
    /// Keys 1/2/3 switch the broad phase (brute force, sweep and prune, uniform grid) to compare the number of tests
    /// and the time, for the same contacts. Total kinetic energy and momentum are displayed (they must stay constant).
    /// </summary>
    public sealed class DemoBroadPhase : MonoBehaviour
    {
        public int count = 1000;
        public float radius = 0.12f;
        public float boxSize = 10f;
        public BroadPhaseType mode = BroadPhaseType.UniformGrid;

        private Vector3[] pos, vel;
        private readonly List<Aabb> boxes = new List<Aabb>();
        private readonly List<BodyPair> pairs = new List<BodyPair>();
        private IBroadPhase broad;
        private BroadPhaseType current;
        private int contacts;
        private double ms;
        private Matrix4x4[] matrices = new Matrix4x4[1023];
        private Material material;
        private readonly System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

        private void Start()
        {
            pos = new Vector3[count];
            vel = new Vector3[count];
            var rng = new System.Random(1);
            float half = 0.5f * boxSize - radius;
            for (int i = 0; i < count; i++)
            {
                pos[i] = new Vector3((float)(rng.NextDouble() * 2 - 1) * half, (float)(rng.NextDouble() * 2 - 1) * half, (float)(rng.NextDouble() * 2 - 1) * half);
                vel[i] = new Vector3((float)(rng.NextDouble() * 2 - 1), (float)(rng.NextDouble() * 2 - 1), (float)(rng.NextDouble() * 2 - 1)) * 3f;
            }
            material = RenderUtils.CreateLitMaterial(new Color(0.3f, 0.7f, 1f));
            broad = BroadPhaseFactory.Create(mode);
            current = mode;
        }

        private void Update()
        {
            Keyboard kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.digit1Key.wasPressedThisFrame) mode = BroadPhaseType.BruteForce;
                if (kb.digit2Key.wasPressedThisFrame) mode = BroadPhaseType.SweepAndPrune;
                if (kb.digit3Key.wasPressedThisFrame) mode = BroadPhaseType.UniformGrid;
            }
            if (mode != current)
            {
                broad = BroadPhaseFactory.Create(mode);
                current = mode;
            }

            float dt = 1f / 60f;
            float half = 0.5f * boxSize - radius;
            // 1) Motion: p(t + dt) = p + v dt (no gravity). 2) Walls: reflect the normal velocity component.
            for (int i = 0; i < count; i++)
            {
                Vector3 p = pos[i] + vel[i] * dt;
                Vector3 v = vel[i];
                for (int k = 0; k < 3; k++)
                {
                    if (p[k] > half) { p[k] = half; v[k] = -Mathf.Abs(v[k]); }
                    if (p[k] < -half) { p[k] = -half; v[k] = Mathf.Abs(v[k]); }
                }
                pos[i] = p;
                vel[i] = v;
            }

            // 3) Broad phase on the spheres' AABBs, then exact sphere-sphere test and elastic response.
            sw.Restart();
            boxes.Clear();
            Vector3 e = new Vector3(radius, radius, radius);
            for (int i = 0; i < count; i++)
                boxes.Add(new Aabb(pos[i] - e, pos[i] + e));
            broad.FindPairs(boxes, pairs);
            contacts = 0;
            float r2 = 4f * radius * radius;
            for (int k = 0; k < pairs.Count; k++)
            {
                int a = pairs[k].a, b = pairs[k].b;
                Vector3 d = pos[b] - pos[a];
                float dist2 = VMath.LengthSq(d);
                if (dist2 >= r2 || dist2 < 1e-12f) continue;
                contacts++;
                float dist = Mathf.Sqrt(dist2);
                Vector3 n = d / dist;
                // Separate the spheres (equal masses: half each).
                Vector3 corr = n * (0.5f * (2f * radius - dist));
                pos[a] -= corr;
                pos[b] += corr;
                // Elastic collision with equal masses: exchange the normal components (slide 68).
                float vn = VMath.Dot(vel[a] - vel[b], n);
                if (vn > 0f)
                {
                    vel[a] -= n * vn;
                    vel[b] += n * vn;
                }
            }
            ms = sw.Elapsed.TotalMilliseconds;

            Mesh mesh = RenderUtils.SphereMesh;
            var rp = new RenderParams(material);
            for (int start = 0; start < count; start += matrices.Length)
            {
                int n = Mathf.Min(matrices.Length, count - start);
                for (int i = 0; i < n; i++)
                    matrices[i] = RenderUtils.ScaleTranslation(pos[start + i], 2f * radius);
                Graphics.RenderMeshInstanced(rp, mesh, 0, matrices, n);
            }
        }

        private void OnGUI()
        {
            if (pos == null) return;
            float energy = 0f;
            Vector3 momentum = VMath.Zero;
            for (int i = 0; i < count; i++)
            {
                energy += 0.5f * VMath.LengthSq(vel[i]);
                momentum += vel[i];
            }
            GUI.Label(new Rect(10, 10, 900, 22),
                $"[1] brute force  [2] sweep and prune  [3] uniform grid   ->  {broad.Name}: {broad.TestsLastFrame} tests, " +
                $"{contacts} contacts, {ms:F2} ms");
            GUI.Label(new Rect(10, 30, 900, 22), $"Kinetic energy: {energy:F2}   (walls reflect momentum, energy stays constant)");
        }
    }
}
