using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>Solver extension: aerodynamic drag and lift on every cloth triangle (wind = SolverParams.wind).</summary>
    public sealed class AerodynamicsExtension : SolverExtension
    {
        public float drag;
        public float lift;

        /// <summary>
        /// Simple aerodynamic model of cloth (section 8, Keckeisen et al. 2004): every triangle is a thin airfoil.
        /// With v_rel = v_triangle - v_wind and n oriented against the flow,
        /// drag = -k_d A (n . v_rel) v_rel and lift = -k_l A (n . v_rel) |v_rel| (n - (n . v_hat) v_hat),
        /// distributed to the three vertices as velocity changes.
        /// </summary>
        public override void ApplyForces(float h)
        {
            UnifiedSolver s = Solver;
            List<ClothHandle> cloths = s.Cloths;
            if (cloths.Count == 0 || (drag <= 0f && lift <= 0f))
                return;
            Vector3[] positions = s.positions, velocities = s.velocities;
            float[] invMass = s.invMass;
            Vector3 wind = s.parameters.wind;
            for (int c = 0; c < cloths.Count; c++)
            {
                ClothHandle cloth = cloths[c];
                if (!cloth.aerodynamics) continue;
                int[] tris = cloth.triangles;
                for (int t = 0; t + 2 < tris.Length; t += 3)
                {
                    int a = tris[t], b = tris[t + 1], d = tris[t + 2];
                    Vector3 cross = VMath.Cross(positions[b] - positions[a], positions[d] - positions[a]);
                    float twiceArea = VMath.Length(cross);
                    if (twiceArea < 1e-12f) continue;
                    Vector3 n = cross / twiceArea;
                    float area = 0.5f * twiceArea;
                    Vector3 vRel = (velocities[a] + velocities[b] + velocities[d]) / 3f - wind;
                    float speed = VMath.Length(vRel);
                    if (speed < 1e-6f) continue;
                    float nv = VMath.Dot(n, vRel);
                    if (nv < 0f)
                    {
                        n = -n;
                        nv = -nv;
                    }
                    Vector3 vHat = vRel / speed;
                    Vector3 force = vRel * (-drag * area * nv)
                                    + (n - vHat * VMath.Dot(n, vHat)) * (-lift * area * nv * speed);
                    Vector3 perVertex = force * (h / 3f);
                    velocities[a] += Limit(perVertex * invMass[a]);
                    velocities[b] += Limit(perVertex * invMass[b]);
                    velocities[d] += Limit(perVertex * invMass[d]);
                }
            }
        }

        /// <summary>Safety net: the wind never changes a velocity by more than this per sub-step (m/s).</summary>
        public float maxVelocityChange = 2f;

        private Vector3 Limit(Vector3 dv)
        {
            float l2 = VMath.LengthSq(dv);
            float m = maxVelocityChange;
            return l2 > m * m ? dv * (m / Mathf.Sqrt(l2)) : dv;
        }
    }

    /// <summary>Put next to the <see cref="UnifiedWorld"/> to enable cloth aerodynamics.</summary>
    [RequireComponent(typeof(UnifiedWorld))]
    public sealed class ClothAerodynamics : MonoBehaviour
    {
        [Tooltip("Drag coefficient k_d.")]
        public float drag = 1f;
        [Tooltip("Lift coefficient k_l.")]
        public float lift = 0.5f;

        private UnifiedWorld world;
        private AerodynamicsExtension model;

        private void OnEnable()
        {
            world = GetComponent<UnifiedWorld>();
            world.BeforeStep += Sync;
        }

        private void OnDisable()
        {
            if (world != null) world.BeforeStep -= Sync;
        }

        private void Sync(UnifiedSolver solver, float h)
        {
            if (model == null || model.Solver != solver)
                model = solver.GetOrAddExtension<AerodynamicsExtension>();
            model.drag = drag;
            model.lift = lift;
        }
    }
}
