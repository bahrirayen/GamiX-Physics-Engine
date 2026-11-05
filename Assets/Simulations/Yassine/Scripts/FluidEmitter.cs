using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    /// <summary>
    /// Fluid source and sink (paper section 7: fluids are free particles, so they can be created and removed at
    /// any time). The nozzle shoots a disc of particles on the fluid lattice each time the stream has travelled one
    /// rest spacing, so the jet enters at the rest density. The drain deletes the particles of this emitter that
    /// enter its box, which keeps a steady flow with a bounded particle count.
    /// Particles are added and removed in <see cref="EndSubStep"/>, between two sub-steps, where the solver arrays
    /// are not in use.
    /// </summary>
    public sealed class FluidEmitter : SolverExtension
    {
        public Vector3 nozzle = new Vector3(0f, 1.5f, 0f);
        public Vector3 direction = new Vector3(1f, 0f, 0f);
        public float speed = 2.5f;
        /// <summary>Radius of the nozzle disc.</summary>
        public float nozzleRadius = 0.12f;
        public float particleMass = 1f;
        /// <summary>The emitter stops when this many of its particles are alive.</summary>
        public int maxParticles = 2000;
        public bool emitting = true;

        /// <summary>Drain box (world AABB). Its particles that enter it are removed. Disabled when size is zero.</summary>
        public Vector3 drainCenter;
        public Vector3 drainSize;

        /// <summary>Phase group of the emitted fluid (for rendering).</summary>
        public int Group { get; private set; } = -1;
        public int Alive => alive.Count;
        public int TotalEmitted { get; private set; }
        public int TotalDrained { get; private set; }

        private readonly HashSet<int> alive = new HashSet<int>();
        private readonly List<Vector3> discOffsets = new List<Vector3>();
        private readonly List<int> toRemove = new List<int>();
        private float travelled;

        public override void OnAdded()
        {
            Group = Solver.NewGroup();
        }

        /// <summary>Indices of the living particles of this emitter.</summary>
        public IEnumerable<int> Particles => alive;

        public override void EndSubStep(float h)
        {
            Drain();
            Emit(h);
        }

        private void Drain()
        {
            if (drainSize.x <= 0f || drainSize.y <= 0f || drainSize.z <= 0f || alive.Count == 0)
                return;
            UnifiedSolver s = Solver;
            Vector3 min = drainCenter - drainSize * 0.5f, max = drainCenter + drainSize * 0.5f;
            toRemove.Clear();
            foreach (int i in alive)
            {
                Vector3 p = s.predicted[i];
                if (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z)
                    toRemove.Add(i);
            }
            foreach (int i in toRemove)
            {
                s.RemoveParticle(i);
                alive.Remove(i);
            }
            TotalDrained += toRemove.Count;
        }

        private void Emit(float h)
        {
            if (!emitting || speed <= 0f)
                return;
            UnifiedSolver s = Solver;
            float spacing = s.parameters.FluidRestDistance;
            BuildDisc(spacing);
            travelled += speed * h;
            Vector3 dir = VMath.NormalizeOr(direction, VMath.Right);
            Vector3 v = dir * speed;
            int phase = Phase.Make(Group, Phase.Fluid);
            float w = particleMass > 0f ? 1f / particleMass : 0f;
            // One layer per rest spacing travelled; the leftover distance places the layer along the stream.
            while (travelled >= spacing)
            {
                travelled -= spacing;
                if (alive.Count + discOffsets.Count > maxParticles)
                {
                    travelled = 0f;
                    return;
                }
                Vector3 along = dir * travelled;
                foreach (Vector3 o in discOffsets)
                {
                    int i = s.AddParticle(nozzle + along + o, v, w, phase);
                    alive.Add(i);
                }
                TotalEmitted += discOffsets.Count;
            }
        }

        /// <summary>Lattice points of the nozzle disc, in the plane perpendicular to the direction.</summary>
        private void BuildDisc(float spacing)
        {
            if (discOffsets.Count > 0)
                return;
            VMath.OrthonormalBasis(VMath.NormalizeOr(direction, VMath.Right), out Vector3 t1, out Vector3 t2);
            int m = Mathf.CeilToInt(nozzleRadius / spacing);
            float r2 = nozzleRadius * nozzleRadius + 1e-6f;
            for (int a = -m; a <= m; a++)
            for (int b = -m; b <= m; b++)
            {
                Vector3 o = t1 * (a * spacing) + t2 * (b * spacing);
                if (VMath.LengthSq(o) <= r2)
                    discOffsets.Add(o);
            }
        }
    }
}
