using System;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    public sealed partial class UnifiedSolver
    {
        private SphKernels kernels;
        private float restDensity;
        private float densityDenominator;
        private float[] vorticityMagnitude = new float[0];

        /// <summary>SPH kernels of the fluid (null until the first fluid particle is simulated).</summary>
        public SphKernels Kernels => kernels;
        /// <summary>Spatial hash of the predicted positions, built at the start of each sub-step.</summary>
        public HashGrid Grid => grid;

        /// <summary>
        /// Rest density rho0 and the reference denominator of eq. 5 are computed once on a fully filled lattice of
        /// fluid particles (section 7.1.1 pre-computes the denominator on such a configuration). The solver uses the
        /// per-particle denominator plus a fraction of this reference value as relaxation. Masses are 1 in kernel
        /// units; heavier/lighter particles are handled by the mass-weighted correction.
        /// </summary>
        private void PrepareFluidConstants()
        {
            if (fluidCount <= 0)
                return;
            float h = parameters.KernelRadius;
            if (kernels != null && Mathf.Abs(kernels.h - h) < 1e-7f)
                return;
            kernels = new SphKernels(h);
            float s = parameters.FluidRestDistance;
            int m = Mathf.CeilToInt(h / s);

            float rho = 0f;
            for (int x = -m; x <= m; x++)
            for (int y = -m; y <= m; y++)
            for (int z = -m; z <= m; z++)
            {
                Vector3 p = new Vector3(x, y, z) * s;
                rho += kernels.Poly6(VMath.LengthSq(p));
            }
            restDensity = rho;

            float sumGrad2 = 0f;
            Vector3 sumGrad = VMath.Zero;
            for (int x = -m; x <= m; x++)
            for (int y = -m; y <= m; y++)
            for (int z = -m; z <= m; z++)
            {
                if (x == 0 && y == 0 && z == 0) continue;
                Vector3 r = new Vector3(x, y, z) * -s;
                Vector3 g = kernels.SpikyGradient(r, VMath.Length(r)) / restDensity;
                sumGrad2 += VMath.LengthSq(g);
                sumGrad += g;
            }
            densityDenominator = sumGrad2 + VMath.LengthSq(sumGrad);
        }

        /// <summary>
        /// Fluid density constraint (position based fluids, eq. 26-27): rho_i = sum_fluid W + s sum_solid W,
        /// C_i = rho_i / rho0 - 1 clamped to C_i &gt;= 0 (unilateral: it only separates particles),
        /// lambda_i = -C_i / denominator, dx_i = 1/rho0 sum_j (lambda_i + lambda_j) grad W(x_i - x_j),
        /// scaled by w_i / w_ref so that particles of different masses give buoyancy (section 7.1.1).
        /// Particle-centric gather: every particle writes only its own delta, so both passes run in parallel.
        /// </summary>
        private void SolveDensityConstraints()
        {
            ParallelForRange(Count, densityLambdaRange);
            ClearDeltas();
            ParallelForRange(Count, densityDeltaRange);
            ApplyDeltas(parameters.relaxation);
        }

        private Action<int, int> densityLambdaRange, densityDeltaRange;

        private Action<int, int> fluidVorticityRange, fluidNormalRange, fluidVelocityRange;

        private void CacheFluidDelegates()
        {
            densityLambdaRange = ComputeDensityLambdaRange;
            densityDeltaRange = ComputeDensityDeltaRange;
            fluidVorticityRange = (b, e) => { for (int i = b; i < e; i++) ComputeVorticityAndGradients(i); };
            fluidNormalRange = (b, e) => { for (int i = b; i < e; i++) ComputeSurfaceNormal(i); };
            fluidVelocityRange = (b, e) => { for (int i = b; i < e; i++) UpdateFluidVelocity(i); };
        }

        // The two density passes are the hottest loops of a fluid scene. They are written with plain floats
        // (no Vector3 operators, kernels inlined) because Unity's Mono runtime does not inline small struct methods.

        private void ComputeDensityLambdaRange(int begin, int end)
        {
            float h = kernels.h, h2 = h * h;
            float poly6 = kernels.poly6Coeff, spiky = kernels.spikyGradCoeff;
            float s = parameters.solidDensityScale;
            float invRho0 = 1f / restDensity;
            float relax = densityDenominator * parameters.fluidRelaxation;
            bool unilateral = parameters.unilateralDensity;
            float selfW = poly6 * h2 * h2 * h2;
            for (int i = begin; i < end; i++)
            {
                if (!active[i] || (phases[i] & Phase.Fluid) == 0)
                    continue;
                Vector3 xi = predicted[i];
                float xx = xi.x, xy = xi.y, xz = xi.z;
                float rho = selfW;
                float sumGrad2 = 0f, gx = 0f, gy = 0f, gz = 0f;
                int n = neighborCounts[i];
                int nb = i * maxNeighbors;
                for (int k = 0; k < n; k++)
                {
                    int j = neighbors[nb + k];
                    Vector3 xj = predicted[j];
                    float rx = xx - xj.x, ry = xy - xj.y, rz = xz - xj.z;
                    float r2 = rx * rx + ry * ry + rz * rz;
                    if (r2 >= h2) continue;
                    float d = h2 - r2;
                    bool fluid = (phases[j] & Phase.Fluid) != 0;
                    float weight = fluid ? 1f : s;
                    rho += poly6 * d * d * d * weight;
                    float len = (float)Math.Sqrt(r2);
                    if (len < 1e-9f) continue;
                    float hl = h - len;
                    // Spiky gradient / rho0: spiky (h - r)^2 r / |r|
                    float c = spiky * hl * hl / len * weight * invRho0;
                    float ax = rx * c, ay = ry * c, az = rz * c;
                    gx += ax; gy += ay; gz += az;
                    if (fluid)
                        sumGrad2 += ax * ax + ay * ay + az * az; // solid particles are not moved by this constraint
                }
                densities[i] = rho;
                float C = rho * invRho0 - 1f;
                if (unilateral && C < 0f)
                    C = 0f;
                // Eq. 5 with the actual gradients of this particle. The reference value (filled lattice) is added as
                // relaxation: under strong compression the gradients grow, and a fixed denominator would overshoot.
                float denom = sumGrad2 + gx * gx + gy * gy + gz * gz + relax;
                lambdas[i] = denom > 1e-12f ? -C / denom : 0f;
            }
        }

        private void ComputeDensityDeltaRange(int begin, int end)
        {
            float h = kernels.h, h2 = h * h;
            float spiky = kernels.spikyGradCoeff;
            float s = parameters.solidDensityScale;
            float invRho0 = 1f / restDensity;
            float refInv = fluidReferenceInvMass;
            for (int i = begin; i < end; i++)
            {
                if (!active[i] || (phases[i] & Phase.Fluid) == 0 || invMass[i] == 0f)
                    continue;
                Vector3 xi = predicted[i];
                float xx = xi.x, xy = xi.y, xz = xi.z;
                float li = lambdas[i];
                float dx = 0f, dy = 0f, dz = 0f;
                int n = neighborCounts[i];
                int nb = i * maxNeighbors;
                for (int k = 0; k < n; k++)
                {
                    int j = neighbors[nb + k];
                    Vector3 xj = predicted[j];
                    float rx = xx - xj.x, ry = xy - xj.y, rz = xz - xj.z;
                    float r2 = rx * rx + ry * ry + rz * rz;
                    if (r2 >= h2 || r2 < 1e-18f) continue;
                    float len = (float)Math.Sqrt(r2);
                    float hl = h - len;
                    float lambdaSum = (phases[j] & Phase.Fluid) != 0 ? li + lambdas[j] : li * s;
                    float c = spiky * hl * hl / len * lambdaSum;
                    dx += rx * c; dy += ry * c; dz += rz * c;
                }
                float scale = (refInv > 0f ? invMass[i] / refInv : 1f) * invRho0;
                deltas[i] = new Vector3(dx * scale, dy * scale, dz * scale);
                deltaCounts[i] = 1;
            }
        }

        // ---------------------------------------------------------------- internal forces (Algorithm 1, line 26)

        private float stepDt;

        /// <summary>
        /// Velocity-level effects of fluids, computed from the post-solve velocities:
        /// XSPH viscosity, vorticity confinement, Akinci cohesion and surface tension.
        /// </summary>
        private void ApplyFluidInternalForces(float h)
        {
            stepDt = h;
            SolverParams p = parameters;
            bool needVorticity = p.vorticityConfinement > 0f;
            bool needNormals = p.surfaceTension > 0f;
            if (needVorticity)
            {
                if (vorticityMagnitude.Length < positions.Length)
                    vorticityMagnitude = new float[positions.Length];
                ParallelForRange(Count, fluidVorticityRange);
            }
            if (needNormals)
                ParallelForRange(Count, fluidNormalRange);
            ParallelForRange(Count, fluidVelocityRange);

            for (int i = 0; i < Count; i++)
            {
                if (!active[i] || !Phase.IsFluid(phases[i]) || invMass[i] == 0f) continue;
                velocities[i] = velocityBuffer[i];
            }
        }

        /// <summary>
        /// Vorticity (curl of the velocity) omega_i = sum_j V_j (v_j - v_i) x grad_j W with the SPH volume V_j = 1/rho_j,
        /// so omega is in 1/s whatever the kernel size. Stored in scratchVectors.
        /// </summary>
        private void ComputeVorticityAndGradients(int i)
        {
            if (!active[i] || !Phase.IsFluid(phases[i]))
            {
                scratchVectors[i] = VMath.Zero;
                vorticityMagnitude[i] = 0f;
                return;
            }
            float h = kernels.h;
            Vector3 xi = predicted[i], vi = velocities[i];
            Vector3 omega = VMath.Zero;
            int n = neighborCounts[i];
            int baseIndex = i * maxNeighbors;
            for (int k = 0; k < n; k++)
            {
                int j = neighbors[baseIndex + k];
                if (!Phase.IsFluid(phases[j])) continue;
                Vector3 r = xi - predicted[j];
                float len = VMath.Length(r);
                if (len >= h) continue;
                // grad with respect to x_j is -grad W(x_i - x_j).
                float rhoJ = densities[j] > 1e-6f ? densities[j] : restDensity;
                omega += VMath.Cross(velocities[j] - vi, -kernels.SpikyGradient(r, len)) / rhoJ;
            }
            scratchVectors[i] = omega;
            vorticityMagnitude[i] = VMath.Length(omega);
        }

        /// <summary>
        /// Unnormalized surface normal n_i = sum_j (m_j / rho_j) grad W(x_i - x_j), large near the surface.
        /// Stored in scratchVectors2 (multiplied by h, dimensionless).
        /// </summary>
        private void ComputeSurfaceNormal(int i)
        {
            if (!active[i] || !Phase.IsFluid(phases[i]))
            {
                scratchVectors2[i] = VMath.Zero;
                return;
            }
            float h = kernels.h;
            Vector3 xi = predicted[i];
            Vector3 normal = VMath.Zero;
            int n = neighborCounts[i];
            int baseIndex = i * maxNeighbors;
            for (int k = 0; k < n; k++)
            {
                int j = neighbors[baseIndex + k];
                if (!Phase.IsFluid(phases[j])) continue;
                Vector3 r = xi - predicted[j];
                float len = VMath.Length(r);
                if (len >= h) continue;
                float rhoJ = densities[j] > 1e-6f ? densities[j] : restDensity;
                normal += kernels.SpikyGradient(r, len) / rhoJ;
            }
            scratchVectors2[i] = normal * h;
        }

        private void UpdateFluidVelocity(int i)
        {
            if (!active[i] || !Phase.IsFluid(phases[i]) || invMass[i] == 0f)
                return;
            SolverParams p = parameters;
            float h = kernels.h, h2 = h * h;
            float dtLocal = stepDt;
            Vector3 xi = predicted[i], vi = velocities[i];
            float rhoI = densities[i] > 1e-6f ? densities[i] : restDensity;
            int phaseGroup = Phase.Group(phases[i]);

            bool doCohesion = p.cohesion > 0f || p.surfaceTension > 0f;
            bool doVorticity = p.vorticityConfinement > 0f;
            bool needGradient = doCohesion || doVorticity;
            float poly6 = kernels.poly6Coeff;

            float visX = 0f, visY = 0f, visZ = 0f;
            float cohX = 0f, cohY = 0f, cohZ = 0f;
            float curX = 0f, curY = 0f, curZ = 0f;
            float etaX = 0f, etaY = 0f, etaZ = 0f;
            int sameCount = 0;
            Vector3 omegaI = scratchVectors[i];
            Vector3 normalI = scratchVectors2[i];
            float xx = xi.x, xy = xi.y, xz = xi.z;
            float vx = vi.x, vy = vi.y, vz = vi.z;
            float spiky = kernels.spikyGradCoeff;
            float halfH = 0.5f * h;
            float h6 = h2 * h2 * h2;
            float cohesionNorm = 64f / h6; // the Akinci spline peaks at h^6/64 (r = h/2)
            float twoRho0 = 2f * restDensity;

            int n = neighborCounts[i];
            int baseIndex = i * maxNeighbors;
            for (int k = 0; k < n; k++)
            {
                int j = neighbors[baseIndex + k];
                int phaseJ = phases[j];
                if ((phaseJ & Phase.Fluid) == 0) continue;
                Vector3 xj = predicted[j];
                float rx = xx - xj.x, ry = xy - xj.y, rz = xz - xj.z;
                float r2 = rx * rx + ry * ry + rz * rz;
                if (r2 >= h2) continue;
                float d = h2 - r2;
                float w = poly6 * d * d * d;
                Vector3 vj = velocities[j];
                float ux = vj.x - vx, uy = vj.y - vy, uz = vj.z - vz;

                // XSPH viscosity: v_i += c sum (v_j - v_i) W / rho_i.
                visX += ux * w; visY += uy * w; visZ += uz * w;

                if (!needGradient) continue;
                float len = (float)Math.Sqrt(r2);
                if (len <= 1e-9f) continue;
                float rhoJ = densities[j] > 1e-6f ? densities[j] : restDensity;
                float hl = h - len;
                float gs = spiky * hl * hl / len; // spiky gradient = gs * r
                if (doVorticity)
                {
                    float c = gs * vorticityMagnitude[j] / rhoJ;
                    etaX += rx * c; etaY += ry * c; etaZ += rz * c;
                }
                if (doCohesion && (phaseJ & Phase.GroupMask) == phaseGroup)
                {
                    // Akinci 2013: cohesion and curvature only between particles of the same phase.
                    float kij = twoRho0 / (rhoI + rhoJ);
                    float hr3 = hl * hl * hl, r3 = len * len * len;
                    float spline = len > halfH ? hr3 * r3 : 2f * hr3 * r3 - h6 / 64f;
                    float c = kij * spline * cohesionNorm / len;
                    cohX -= rx * c; cohY -= ry * c; cohZ -= rz * c;
                    Vector3 nj = scratchVectors2[j];
                    curX -= (normalI.x - nj.x) * kij; curY -= (normalI.y - nj.y) * kij; curZ -= (normalI.z - nj.z) * kij;
                    sameCount++;
                }
            }
            Vector3 cohesion = new Vector3(cohX, cohY, cohZ);
            Vector3 curvature = new Vector3(curX, curY, curZ);
            Vector3 eta = new Vector3(etaX, etaY, etaZ);
            Vector3 viscous = new Vector3(visX, visY, visZ);

            Vector3 v = vi + viscous * (p.viscosity / rhoI);

            if (sameCount > 0)
            {
                // Strengths are accelerations in units of g, independent of the time step and particle size.
                float g = VMath.Length(p.gravity);
                float accel = (g > 1e-3f ? g : 9.81f) * dtLocal;
                if (p.cohesion > 0f)
                    v += cohesion * (p.cohesion * accel);
                if (p.surfaceTension > 0f)
                    v += curvature * (p.surfaceTension * accel);
            }

            if (p.vorticityConfinement > 0f)
            {
                // f = epsilon h (N x omega) (Fedkiw et al. 2001), N = eta / |eta| points toward higher vorticity.
                Vector3 nVort = VMath.Normalize(eta);
                v += VMath.Cross(nVort, omegaI) * (p.vorticityConfinement * h * dtLocal);
            }

            velocityBuffer[i] = v;
        }
    }
}
