using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    public enum FluidColorMode
    {
        /// <summary>Each fluid block in its own color (shows how phases mix).</summary>
        Phase,
        /// <summary>Speed, from blue (still) to red (fast).</summary>
        Speed,
        /// <summary>rho / rho0 from 0.8 (blue, surface) to 1.1 (red, compressed): how well eq. 26 is satisfied.</summary>
        Density
    }

    /// <summary>
    /// Draws the fluid particles of a <see cref="UnifiedWorld"/> as GPU-instanced spheres. Unlike the library's
    /// ParticleRenderer it can color each particle by speed or density, quantized into a few color bins so every bin
    /// is one instanced batch. Rigid bodies are drawn by their own meshes, so their particles are hidden here.
    /// </summary>
    public sealed class FluidParticleRenderer : MonoBehaviour
    {
        public UnifiedWorld world;
        public FluidColorMode colorMode = FluidColorMode.Phase;
        public Color defaultColor = new Color(0.15f, 0.45f, 0.95f);
        [Tooltip("Speed mapped to the hottest color (m/s).")]
        public float maxSpeed = 3f;
        public float minDensity = 0.8f, maxDensity = 1.1f;
        [Tooltip("Draw solid (non-fluid) particles too.")]
        public bool showSolidParticles;
        [Tooltip("Rendered diameter as a multiple of the fluid rest spacing.")]
        public float sizeScale = 1f;

        private const int Bins = 16;
        private readonly Dictionary<int, Color> groupColors = new Dictionary<int, Color>();
        private readonly List<Matrix4x4>[] gradientBatches = new List<Matrix4x4>[Bins];
        private readonly Dictionary<Color, List<Matrix4x4>> colorBatches = new Dictionary<Color, List<Matrix4x4>>();
        private readonly Color[] gradient = new Color[Bins];
        private readonly Matrix4x4[] buffer = new Matrix4x4[1023];

        public void SetGroupColor(int group, Color c) => groupColors[group] = c;

        public void ClearGroupColors() => groupColors.Clear();

        private void Awake()
        {
            for (int b = 0; b < Bins; b++)
            {
                gradientBatches[b] = new List<Matrix4x4>(512);
                gradient[b] = Heat(b / (float)(Bins - 1));
            }
        }

        /// <summary>Blue - cyan - green - yellow - red.</summary>
        private static Color Heat(float t)
        {
            t = Mathf.Clamp01(t);
            if (t < 0.25f) return Color.Lerp(new Color(0.1f, 0.25f, 0.9f), new Color(0.1f, 0.8f, 0.95f), t / 0.25f);
            if (t < 0.5f) return Color.Lerp(new Color(0.1f, 0.8f, 0.95f), new Color(0.2f, 0.9f, 0.3f), (t - 0.25f) / 0.25f);
            if (t < 0.75f) return Color.Lerp(new Color(0.2f, 0.9f, 0.3f), new Color(0.98f, 0.9f, 0.2f), (t - 0.5f) / 0.25f);
            return Color.Lerp(new Color(0.98f, 0.9f, 0.2f), new Color(0.95f, 0.2f, 0.1f), (t - 0.75f) / 0.25f);
        }

        private void LateUpdate()
        {
            UnifiedSolver s = world != null ? world.Solver : null;
            if (s == null) return;

            foreach (List<Matrix4x4> l in gradientBatches) l.Clear();
            foreach (List<Matrix4x4> l in colorBatches.Values) l.Clear();

            float dFluid = s.parameters.FluidRestDistance * sizeScale;
            float dSolid = 2f * s.parameters.radius * sizeScale;
            float rho0 = s.RestDensity;
            for (int i = 0; i < s.Count; i++)
            {
                if (!s.active[i]) continue;
                int phase = s.phases[i];
                bool fluid = Phase.IsFluid(phase);
                if (!fluid && !showSolidParticles) continue;
                Matrix4x4 m = RenderUtils.ScaleTranslation(s.positions[i], fluid ? dFluid : dSolid);

                if (fluid && colorMode != FluidColorMode.Phase)
                {
                    float t = colorMode == FluidColorMode.Speed
                        ? VMath.Length(s.velocities[i]) / Mathf.Max(1e-3f, maxSpeed)
                        : (rho0 > 0f ? (s.densities[i] / rho0 - minDensity) / Mathf.Max(1e-3f, maxDensity - minDensity) : 0f);
                    int bin = Mathf.Clamp((int)(Mathf.Clamp01(t) * (Bins - 1) + 0.5f), 0, Bins - 1);
                    gradientBatches[bin].Add(m);
                    continue;
                }
                if (!groupColors.TryGetValue(Phase.Group(phase), out Color c))
                    c = fluid ? defaultColor : Color.gray;
                if (!colorBatches.TryGetValue(c, out List<Matrix4x4> list))
                {
                    list = new List<Matrix4x4>(1024);
                    colorBatches[c] = list;
                }
                list.Add(m);
            }

            for (int b = 0; b < Bins; b++)
                Draw(gradient[b], gradientBatches[b]);
            foreach (KeyValuePair<Color, List<Matrix4x4>> kv in colorBatches)
                Draw(kv.Key, kv.Value);
        }

        private void Draw(Color color, List<Matrix4x4> list)
        {
            if (list.Count == 0) return;
            var rp = new RenderParams(RenderUtils.GetSharedMaterial(color));
            Mesh mesh = RenderUtils.SphereMesh;
            for (int start = 0; start < list.Count; start += buffer.Length)
            {
                int n = Mathf.Min(buffer.Length, list.Count - start);
                list.CopyTo(start, buffer, 0, n);
                Graphics.RenderMeshInstanced(rp, mesh, 0, buffer, n);
            }
        }
    }
}
