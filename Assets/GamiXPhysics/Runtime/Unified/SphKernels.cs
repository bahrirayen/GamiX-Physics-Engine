using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// SPH smoothing kernels of support radius h (Mueller et al. 2003), used by position based fluids
    /// (density constraint, eq. 26-27), smoke advection (eq. 28) and the gas turbulence model (eq. 30-32).
    /// </summary>
    public sealed class SphKernels
    {
        public readonly float h;
        private readonly float h2;
        internal readonly float poly6Coeff;
        internal readonly float spikyGradCoeff;
        private readonly float cohesionCoeff;
        private readonly float cohesionOffset;

        public SphKernels(float h)
        {
            this.h = h;
            h2 = h * h;
            float h3 = h2 * h, h6 = h3 * h3, h9 = h6 * h3;
            poly6Coeff = 315f / (64f * Mathf.PI * h9);
            spikyGradCoeff = -45f / (Mathf.PI * h6);
            cohesionCoeff = 32f / (Mathf.PI * h9);
            cohesionOffset = h6 / 64f;
        }

        /// <summary>Poly6: W(r, h) = 315 / (64 pi h^9) (h^2 - r^2)^3, for r &lt; h.</summary>
        public float Poly6(float r2)
        {
            if (r2 >= h2) return 0f;
            float d = h2 - r2;
            return poly6Coeff * d * d * d;
        }

        /// <summary>Gradient of the spiky kernel: -45 / (pi h^6) (h - r)^2 r/|r|. Non-zero at r -> 0 (good for pressure).</summary>
        public Vector3 SpikyGradient(Vector3 r, float len)
        {
            if (len >= h || len < 1e-9f) return VMath.Zero;
            float d = h - len;
            return r * (spikyGradCoeff * d * d / len);
        }

        /// <summary>
        /// Akinci 2013 cohesion spline C(r): attraction at mid range, repulsion near r = 0.
        /// Normalized so that its maximum is 1 (the solver uses a dimensionless strength).
        /// </summary>
        public float CohesionNormalized(float r)
        {
            if (r >= h || r <= 0f) return 0f;
            float c;
            float hr = h - r;
            if (2f * r > h)
                c = cohesionCoeff * hr * hr * hr * r * r * r;
            else
                c = cohesionCoeff * (2f * hr * hr * hr * r * r * r - cohesionOffset);
            // Max of the spline is at r = h/2: value cohesionCoeff * h^6 / 64.
            return c / (cohesionCoeff * cohesionOffset);
        }
    }
}
