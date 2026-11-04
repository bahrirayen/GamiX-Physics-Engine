using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Unified particle solver of Macklin et al. 2014 ("Unified Particle Physics for Real-Time Applications").
    /// Every object (fluid, granular material, rigid body, cloth, rope...) is made of particles
    /// (x, v, inverse mass, phase) coupled by position-level constraints, solved with a parallel Jacobi iteration
    /// where each particle's correction is averaged over the constraints acting on it (eq. 12) and over-relaxed (eq. 13).
    /// The step follows Algorithm 1 of the paper. Simulations add their own constraints and forces through
    /// <see cref="SolverExtension"/>.
    /// </summary>
    public sealed partial class UnifiedSolver
    {
        public readonly SolverParams parameters;

        // ---------------------------------------------------------------- particle state (section 3)
        /// <summary>Number of particle slots in use (some may be inactive).</summary>
        public int Count { get; private set; }
        public int ActiveCount { get; private set; }

        /// <summary>x: positions at the start of the step (and final positions after it).</summary>
        public Vector3[] positions = new Vector3[0];
        /// <summary>x*: predicted / candidate positions during the step.</summary>
        public Vector3[] predicted = new Vector3[0];
        public Vector3[] velocities = new Vector3[0];
        /// <summary>Rest positions (used to filter self collisions of cloth/soft groups).</summary>
        public Vector3[] restPositions = new Vector3[0];
        /// <summary>w = 1/m (0 = pinned / infinite mass).</summary>
        public float[] invMass = new float[0];
        public int[] phases = new int[0];
        public bool[] active = new bool[0];
        /// <summary>Index of the rigid group a particle belongs to, or -1.</summary>
        public int[] rigidOf = new int[0];
        /// <summary>Index of the particle inside its rigid group.</summary>
        public int[] rigidSlot = new int[0];
        /// <summary>Fluid density of the last solve (also used by gas drag and smoke).</summary>
        public float[] densities = new float[0];

        private float[] contactInvMass = new float[0];
        private Vector3[] deltas = new Vector3[0];
        private int[] deltaCounts = new int[0];
        private float[] lambdas = new float[0];
        private Vector3[] scratchVectors = new Vector3[0];
        private Vector3[] scratchVectors2 = new Vector3[0];
        private Vector3[] velocityBuffer = new Vector3[0];

        // ---------------------------------------------------------------- neighbours
        private readonly HashGrid grid = new HashGrid();
        private int[] neighbors = new int[0];
        private int[] neighborCounts = new int[0];
        private int maxNeighbors;
        private float searchRadius;

        // ---------------------------------------------------------------- constraints and objects
        public readonly StructList<DistanceConstraint> DistanceConstraints = new StructList<DistanceConstraint>(256);
        public readonly List<RigidGroup> Rigids = new List<RigidGroup>();
        /// <summary>Triangle meshes made of particles (cloth, inflatables), for extensions and rendering.</summary>
        public readonly List<ClothHandle> Cloths = new List<ClothHandle>();
        public readonly List<KinematicShape> Shapes = new List<KinematicShape>();

        private readonly List<SolverExtension> extensions = new List<SolverExtension>();
        /// <summary>Extensions plugged into the step (see <see cref="SolverExtension"/>).</summary>
        public IReadOnlyList<SolverExtension> Extensions => extensions;

        // ---------------------------------------------------------------- statistics
        public double LastStepMilliseconds { get; private set; }
        public float time;

        private readonly Stack<int> freeSlots = new Stack<int>();
        private int nextGroup = 1;
        private int fluidCount;
        private float fluidReferenceInvMass;

        private float dt;
        private readonly Stopwatch stopwatch = new Stopwatch();

        public UnifiedSolver(SolverParams parameters = null)
        {
            this.parameters = parameters ?? new SolverParams();
            CacheDelegates();
            CacheContactDelegates();
            CacheFluidDelegates();
        }

        // ================================================================ particle management

        /// <summary>Returns a new group id for <see cref="Phase.Make"/>.</summary>
        public int NewGroup() => nextGroup++;

        private void EnsureCapacity(int n)
        {
            if (positions.Length >= n)
                return;
            int cap = Math.Max(n, Math.Max(64, positions.Length * 2));
            Array.Resize(ref positions, cap);
            Array.Resize(ref predicted, cap);
            Array.Resize(ref velocities, cap);
            Array.Resize(ref restPositions, cap);
            Array.Resize(ref invMass, cap);
            Array.Resize(ref phases, cap);
            Array.Resize(ref active, cap);
            Array.Resize(ref rigidOf, cap);
            Array.Resize(ref rigidSlot, cap);
            Array.Resize(ref densities, cap);
            Array.Resize(ref contactInvMass, cap);
            Array.Resize(ref deltas, cap);
            Array.Resize(ref deltaCounts, cap);
            Array.Resize(ref lambdas, cap);
            Array.Resize(ref scratchVectors, cap);
            Array.Resize(ref scratchVectors2, cap);
            Array.Resize(ref velocityBuffer, cap);
        }

        /// <summary>Adds a particle and returns its index. Pass invMass = 0 for a pinned particle.</summary>
        public int AddParticle(Vector3 position, Vector3 velocity, float inverseMass, int phase)
        {
            int i;
            if (freeSlots.Count > 0)
            {
                i = freeSlots.Pop();
            }
            else
            {
                EnsureCapacity(Count + 1);
                i = Count++;
            }
            positions[i] = position;
            predicted[i] = position;
            velocities[i] = velocity;
            restPositions[i] = position;
            invMass[i] = inverseMass;
            phases[i] = phase;
            active[i] = true;
            rigidOf[i] = -1;
            rigidSlot[i] = -1;
            densities[i] = 0f;
            ActiveCount++;
            if (Phase.IsFluid(phase))
            {
                fluidCount++;
                if (inverseMass > fluidReferenceInvMass)
                    fluidReferenceInvMass = inverseMass;
            }
            return i;
        }

        /// <summary>
        /// Deactivates a particle so its slot can be reused. Only use it for particles without constraints
        /// (e.g. emitted fluid or gas particles).
        /// </summary>
        public void RemoveParticle(int i)
        {
            if (!active[i])
                return;
            active[i] = false;
            if (Phase.IsFluid(phases[i]))
                fluidCount--;
            ActiveCount--;
            freeSlots.Push(i);
        }

        /// <summary>Pins (infinite mass) or releases a particle.</summary>
        public void SetInverseMass(int i, float w)
        {
            invMass[i] = w;
        }

        /// <summary>Moves a pinned particle (kinematic attachment, e.g. a rope tied to a moving object).</summary>
        public void MovePinnedParticle(int i, Vector3 newPosition)
        {
            positions[i] = newPosition;
            predicted[i] = newPosition;
        }

        /// <summary>Radial velocity kick (explosions, impacts).</summary>
        public void AddRadialImpulse(Vector3 center, float radius, float speedChange)
        {
            float r2 = radius * radius;
            for (int i = 0; i < Count; i++)
            {
                if (!active[i] || invMass[i] == 0f) continue;
                Vector3 d = positions[i] - center;
                float l2 = VMath.LengthSq(d);
                if (l2 > r2 || l2 < 1e-12f) continue;
                float l = Mathf.Sqrt(l2);
                velocities[i] += d / l * (speedChange * (1f - l / radius));
            }
        }

        public int AddShape(KinematicShape shape)
        {
            shape.previousPosition = shape.position;
            shape.previousRotation = shape.rotation;
            Shapes.Add(shape);
            return Shapes.Count - 1;
        }

        public void AddDistanceConstraint(int i, int j, float stiffness = 1f, float tearRatio = 0f, int kind = 0)
        {
            DistanceConstraints.Add(new DistanceConstraint
            {
                i = i, j = j, restLength = VMath.Distance(positions[i], positions[j]),
                stiffness = stiffness, tearRatio = tearRatio, kind = kind
            });
        }

        // ================================================================ extensions

        /// <summary>Plugs an extension into the step. Returns it for chaining.</summary>
        public T AddExtension<T>(T extension) where T : SolverExtension
        {
            if (extension.Solver != null && extension.Solver != this)
                throw new InvalidOperationException("The extension already belongs to another solver.");
            if (extensions.Contains(extension))
                return extension;
            extension.Solver = this;
            extensions.Add(extension);
            extension.OnAdded();
            return extension;
        }

        /// <summary>First extension of type T, or null.</summary>
        public T GetExtension<T>() where T : SolverExtension
        {
            foreach (SolverExtension e in extensions)
                if (e is T t)
                    return t;
            return null;
        }

        /// <summary>The extension of type T, created and added the first time.</summary>
        public T GetOrAddExtension<T>() where T : SolverExtension, new()
        {
            return GetExtension<T>() ?? AddExtension(new T());
        }

        public bool RemoveExtension(SolverExtension extension)
        {
            if (!extensions.Remove(extension))
                return false;
            extension.Solver = null;
            return true;
        }

        /// <summary>
        /// Adds a position correction for particle i during <see cref="SolverExtension.SolveConstraints"/>.
        /// Corrections are averaged per particle (eq. 12). Not thread safe: in a parallel loop, only write
        /// corrections of the particle the loop iteration owns.
        /// </summary>
        public void AddCorrection(int i, Vector3 correction)
        {
            deltas[i] += correction;
            deltaCounts[i]++;
        }

        /// <summary>Length of the particle arrays (extensions size their own per-particle arrays with it).</summary>
        public int Capacity => positions.Length;

        /// <summary>Number of active fluid particles.</summary>
        public int FluidCount => fluidCount;

        /// <summary>Duration of the current sub-step.</summary>
        public float SubStepDuration => dt;

        // ================================================================ parallel helpers

        private Action<int> predictBody, neighborBody;

        private Action<int, int> predictRange, neighborRange;
        private Action<int> shapeMatchingBody;

        private void CacheDelegates()
        {
            predictBody = PredictParticle;
            neighborBody = FindNeighborsOf;
            predictRange = (b, e) => { for (int i = b; i < e; i++) PredictParticle(i); };
            neighborRange = (b, e) => { for (int i = b; i < e; i++) FindNeighborsOf(i); };
            shapeMatchingBody = SolveShapeMatchingGroup;
        }

        private void ParallelFor(int n, Action<int> body)
        {
            if (parameters.multithreaded && n > 256)
            {
                Parallel.For(0, n, body);
            }
            else
            {
                for (int i = 0; i < n; i++)
                    body(i);
            }
        }

        /// <summary>
        /// Runs body(begin, end) over chunks of the range [0, n). One delegate call per chunk instead of one per
        /// particle keeps the overhead low on Unity's Mono runtime.
        /// </summary>
        public void ParallelForRange(int n, Action<int, int> body)
        {
            const int grain = 128;
            if (parameters.multithreaded && n > 2 * grain)
            {
                int chunks = (n + grain - 1) / grain;
                Parallel.For(0, chunks, c =>
                {
                    int begin = c * grain;
                    body(begin, begin + grain < n ? begin + grain : n);
                });
            }
            else
            {
                body(0, n);
            }
        }

        // ================================================================ Algorithm 1

        /// <summary>Advances the simulation by one frame, split in <see cref="SolverParams.substeps"/> sub-steps.</summary>
        public void Step(float frameDt)
        {
            stopwatch.Restart();
            int n = Math.Max(1, parameters.substeps);
            float h = frameDt / n;
            for (int s = 0; s < n; s++)
                SubStep(h);
            LastStepMilliseconds = stopwatch.Elapsed.TotalMilliseconds;
        }

        private void SubStep(float h)
        {
            dt = h;
            SolverParams p = parameters;
            PrepareFluidConstants();

            // Lines 1-5: apply forces, predict positions, mass scaling.
            for (int e = 0; e < extensions.Count; e++)
                extensions[e].ApplyForces(h);
            ParallelForRange(Count, predictRange);

            // Lines 6-9: neighbours and solid contacts.
            maxNeighbors = Math.Max(8, p.maxNeighbors);
            float maxContact = Mathf.Max(p.SolidRestDistance, fluidCount > 0 ? p.FluidRestDistance : 0f);
            searchRadius = Mathf.Max(maxContact + 2f * p.radius * p.collisionMargin, fluidCount > 0 ? p.KernelRadius : 0f);
            if (neighbors.Length < Count * maxNeighbors)
                neighbors = new int[Count * maxNeighbors];
            if (neighborCounts.Length < Count)
                neighborCounts = new int[Count];
            grid.Build(predicted, active, Count, searchRadius);
            ParallelForRange(Count, neighborRange);
            BuildParticleContacts();
            BuildShapeContacts(h);

            // Lines 10-15: pre-stabilization of contacts on the original positions (section 4.4).
            for (int it = 0; it < p.stabilizationIterations; it++)
            {
                ClearDeltas();
                SolveShapeContacts(positions, true);
                SolveParticleContacts(positions, true);
                ApplyDeltasToBoth(1f);
            }

            // Lines 16-22: constraint groups, each solved in parallel (Jacobi) and applied before the next group.
            float omega = p.relaxation;
            for (int it = 0; it < p.iterations; it++)
            {
                if (fluidCount > 0)
                    SolveDensityConstraints();

                ClearDeltas();
                SolveShapeContacts(predicted, false);
                ApplyDeltas(omega);

                ClearDeltas();
                SolveParticleContacts(predicted, false);
                ApplyDeltas(omega);

                if (DistanceConstraints.Count > 0)
                {
                    ClearDeltas();
                    SolveDistanceConstraints(p.iterations);
                    ApplyDeltas(omega);
                }

                // Constraint groups of the extensions (tethers, volumes...).
                for (int e = 0; e < extensions.Count; e++)
                {
                    ClearDeltas();
                    if (extensions[e].SolveConstraints(it))
                        ApplyDeltas(omega);
                }

                if (Rigids.Count > 0)
                {
                    ClearDeltas();
                    SolveShapeMatching();
                    ApplyDeltas(1f);
                }
            }

            // Lines 23-28: velocity update, internal forces, diffuse particles, sleeping.
            float invH = 1f / h;
            for (int i = 0; i < Count; i++)
            {
                if (!active[i]) continue;
                velocities[i] = (predicted[i] - positions[i]) * invH;
            }
            ApplyShapeRestitution();
            if (fluidCount > 0)
                ApplyFluidInternalForces(h);
            for (int e = 0; e < extensions.Count; e++)
                extensions[e].UpdateVelocities(h);
            FinishVelocities(h);
            for (int e = 0; e < extensions.Count; e++)
                extensions[e].EndSubStep(h);

            float sleep2 = p.sleepThreshold * p.sleepThreshold;
            for (int i = 0; i < Count; i++)
            {
                if (!active[i]) continue;
                // Eq. 14: freeze particles whose displacement is below the threshold.
                if (sleep2 > 0f && VMath.DistanceSq(predicted[i], positions[i]) < sleep2)
                    continue;
                positions[i] = predicted[i];
            }
            time += h;
        }

        private void PredictParticle(int i)
        {
            if (!active[i])
                return;
            SolverParams p = parameters;
            if (invMass[i] > 0f)
            {
                Vector3 g = Phase.IsFluid(phases[i]) ? p.gravity * p.fluidGravityScale : p.gravity;
                velocities[i] += g * dt;
                predicted[i] = positions[i] + velocities[i] * dt;
            }
            else
            {
                velocities[i] = VMath.Zero;
                predicted[i] = positions[i];
            }
            // Shock propagation (section 5.2, eq. 21): s = exp(-k h), lower particles are heavier for contacts.
            float w = invMass[i];
            if (p.shockPropagation > 0f && w > 0f)
            {
                float height = Mathf.Max(0f, predicted[i].y - p.groundHeight);
                w *= Mathf.Exp(p.shockPropagation * height);
            }
            contactInvMass[i] = w;
        }

        private void FindNeighborsOf(int i)
        {
            if (!active[i])
            {
                neighborCounts[i] = 0;
                return;
            }
            neighborCounts[i] = grid.Query(predicted[i], searchRadius, predicted, neighbors, i * maxNeighbors, maxNeighbors, i);
        }

        private void ClearDeltas()
        {
            Array.Clear(deltas, 0, Count);
            Array.Clear(deltaCounts, 0, Count);
        }

        /// <summary>Eq. 13: x* += omega / n * sum of the corrections.</summary>
        private void ApplyDeltas(float omega)
        {
            for (int i = 0; i < Count; i++)
            {
                int c = deltaCounts[i];
                if (c > 0)
                    predicted[i] += deltas[i] * (omega / c);
            }
        }

        /// <summary>Pre-stabilization: the corrections move both x and x* (lines 13-14).</summary>
        private void ApplyDeltasToBoth(float omega)
        {
            for (int i = 0; i < Count; i++)
            {
                int c = deltaCounts[i];
                if (c > 0)
                {
                    Vector3 d = deltas[i] * (omega / c);
                    positions[i] += d;
                    predicted[i] += d;
                }
            }
        }

        private void FinishVelocities(float h)
        {
            SolverParams p = parameters;
            float damp = p.damping > 0f ? 1f / (1f + p.damping * h) : 1f;
            float max2 = p.maxSpeed * p.maxSpeed;
            for (int i = 0; i < Count; i++)
            {
                if (!active[i]) continue;
                Vector3 v = velocities[i] * damp;
                float s2 = VMath.LengthSq(v);
                if (s2 > max2)
                    v *= p.maxSpeed / Mathf.Sqrt(s2);
                velocities[i] = v;
            }
        }

        // ================================================================ queries

        /// <summary>Closest particle hit by a ray, particles being spheres of the solver radius (picking).</summary>
        public bool RaycastParticles(Geometry.Ray3D ray, float maxDistance, out int particle, out float t)
        {
            particle = -1;
            t = maxDistance;
            float r = parameters.radius;
            for (int i = 0; i < Count; i++)
            {
                if (!active[i]) continue;
                if (Geometry.Raycast.RaySphere(ray, new Geometry.Sphere(positions[i], r), out Geometry.RayHit hit, t))
                {
                    t = hit.t;
                    particle = i;
                }
            }
            return particle >= 0;
        }

        /// <summary>Neighbours of particle i found at the start of the last sub-step.</summary>
        public int GetNeighbors(int i, out int offset)
        {
            offset = i * maxNeighbors;
            return i < neighborCounts.Length ? neighborCounts[i] : 0;
        }

        public int NeighborAt(int index) => neighbors[index];

        public float RestDensity => restDensity;
    }
}
