using System.Collections.Generic;
using GamiX.Physics.Components;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Physics.Rendering
{
    /// <summary>
    /// Draws the particles of a <see cref="UnifiedWorld"/> as GPU-instanced spheres, one batch per color.
    /// Colors come from the <see cref="ParticleBody"/> that created the particles.
    /// </summary>
    [RequireComponent(typeof(UnifiedWorld))]
    public sealed class ParticleRenderer : MonoBehaviour
    {
        public Color fluidColor = new Color(0.2f, 0.5f, 1f);
        public Color defaultColor = new Color(0.8f, 0.8f, 0.8f);
        [Tooltip("Rendered diameter as a multiple of 2 x particle radius.")]
        public float sizeScale = 1f;

        private UnifiedWorld world;
        private readonly Dictionary<int, Color> groupColors = new Dictionary<int, Color>();
        private readonly HashSet<int> hiddenGroups = new HashSet<int>();
        private readonly Dictionary<Color, List<Matrix4x4>> batches = new Dictionary<Color, List<Matrix4x4>>();
        private Matrix4x4[] buffer = new Matrix4x4[1023];
        private int knownBodies = -1;

        private void Awake()
        {
            world = GetComponent<UnifiedWorld>();
        }

        private void RefreshGroups(UnifiedSolver solver)
        {
            ParticleBody[] bodies = FindObjectsByType<ParticleBody>();
            if (bodies.Length == knownBodies) return;
            knownBodies = bodies.Length;
            groupColors.Clear();
            hiddenGroups.Clear();
            foreach (ParticleBody b in bodies)
            {
                int group = b.ParticleGroup;
                if (group < 0 && b.Particles.Count > 0)
                    group = Phase.Group(solver.phases[b.Particles[0]]);
                else if (group < 0)
                    knownBodies = -1; // not built yet: retry next frame
                if (group < 0) continue;
                groupColors[group] = b.color;
                if (!b.renderParticles) hiddenGroups.Add(group);
            }
        }

        private void LateUpdate()
        {
            UnifiedSolver solver = world != null ? world.Solver : null;
            if (solver == null) return;
            RefreshGroups(solver);

            foreach (List<Matrix4x4> list in batches.Values) list.Clear();
            float d = 2f * solver.parameters.radius * sizeScale;
            float dFluid = solver.parameters.FluidRestDistance * sizeScale;

            for (int i = 0; i < solver.Count; i++)
            {
                if (!solver.active[i]) continue;
                int phase = solver.phases[i];
                int group = Phase.Group(phase);
                if (hiddenGroups.Contains(group)) continue;
                bool fluid = Phase.IsFluid(phase);
                if (!groupColors.TryGetValue(group, out Color c))
                    c = fluid ? fluidColor : defaultColor;
                AddInstance(c, RenderUtils.ScaleTranslation(solver.positions[i], fluid ? dFluid : d));
            }

            Mesh mesh = RenderUtils.SphereMesh;
            foreach (KeyValuePair<Color, List<Matrix4x4>> kv in batches)
            {
                List<Matrix4x4> list = kv.Value;
                if (list.Count == 0) continue;
                var rp = new RenderParams(RenderUtils.GetSharedMaterial(kv.Key));
                for (int start = 0; start < list.Count; start += buffer.Length)
                {
                    int n = Mathf.Min(buffer.Length, list.Count - start);
                    list.CopyTo(start, buffer, 0, n);
                    Graphics.RenderMeshInstanced(rp, mesh, 0, buffer, n);
                }
            }
        }

        private void AddInstance(Color c, Matrix4x4 m)
        {
            if (!batches.TryGetValue(c, out List<Matrix4x4> list))
            {
                list = new List<Matrix4x4>(1024);
                batches[c] = list;
            }
            list.Add(m);
        }
    }
}
