using System;
using System.Collections.Generic;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Physics.Components
{
    /// <summary>
    /// Scene entry point of the unified particle solver. Put one in the scene; every <see cref="ParticleBody"/> and
    /// <see cref="ParticleCollider"/> registers itself. The world steps the solver at a fixed rate from Update.
    /// No Unity physics (Rigidbody, Collider, Physics.*) is involved.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    [DisallowMultipleComponent]
    public sealed class UnifiedWorld : MonoBehaviour
    {
        public SolverParams parameters = new SolverParams();

        [Tooltip("Simulation frames per second (each frame is split in sub-steps).")]
        public float stepRate = 60f;
        [Tooltip("Max simulation frames per rendered frame (avoids the spiral of death).")]
        public int maxStepsPerFrame = 2;
        public bool paused;
        public bool showStats = true;

        public UnifiedSolver Solver { get; private set; }

        /// <summary>Called before each simulation frame (move kinematic objects, add forces...).</summary>
        public event Action<UnifiedSolver, float> BeforeStep;
        /// <summary>Called after each simulation frame.</summary>
        public event Action<UnifiedSolver, float> AfterStep;

        private readonly List<ParticleBody> bodies = new List<ParticleBody>();
        private readonly List<ParticleCollider> colliders = new List<ParticleCollider>();
        private float accumulator;
        private bool started;

        public static UnifiedWorld Find(Component from)
        {
            UnifiedWorld w = from.GetComponentInParent<UnifiedWorld>();
            return w != null ? w : FindAnyObjectByType<UnifiedWorld>();
        }

        private void Awake()
        {
            Solver = new UnifiedSolver(parameters);
        }

        private void Start()
        {
            started = true;
            foreach (ParticleCollider c in colliders) c.Build(Solver);
            foreach (ParticleBody b in bodies) b.Build(Solver);
        }

        internal void Register(ParticleBody body)
        {
            if (bodies.Contains(body)) return;
            bodies.Add(body);
            if (started) body.Build(Solver);
        }

        internal void Register(ParticleCollider collider)
        {
            if (colliders.Contains(collider)) return;
            colliders.Add(collider);
            if (started) collider.Build(Solver);
        }

        internal void Unregister(ParticleCollider collider)
        {
            colliders.Remove(collider);
        }

        private void Update()
        {
            if (paused || Solver == null)
                return;
            float h = 1f / Mathf.Max(1f, stepRate);
            accumulator += Time.deltaTime;
            int steps = 0;
            while (accumulator >= h && steps < maxStepsPerFrame)
            {
                SimulateFrame(h);
                accumulator -= h;
                steps++;
            }
            if (steps == maxStepsPerFrame)
                accumulator = 0f;
            foreach (ParticleBody b in bodies)
                b.SyncVisual(Solver);
        }

        /// <summary>One simulation frame of duration h (can be called manually when paused).</summary>
        public void SimulateFrame(float h)
        {
            foreach (ParticleCollider c in colliders) c.SyncPose(h);
            foreach (ParticleBody b in bodies) b.PreStep(Solver, h);
            BeforeStep?.Invoke(Solver, h);
            Solver.Step(h);
            AfterStep?.Invoke(Solver, h);
        }

        private void OnGUI()
        {
            if (!showStats || Solver == null)
                return;
            GUI.Label(new Rect(10, 10, 600, 22),
                $"Particles: {Solver.ActiveCount}   Contacts: {Solver.ParticleContactCount} + {Solver.ShapeContactCount}   " +
                $"Rigids: {Solver.Rigids.Count}   Solver: {Solver.LastStepMilliseconds:F1} ms");
        }
    }

    /// <summary>Base class of everything made of particles (fluid, granular, rigid, cloth, rope, balloon).</summary>
    public abstract class ParticleBody : MonoBehaviour
    {
        public Color color = new Color(0.3f, 0.6f, 1f);
        [Tooltip("Draw the particles of this body with the world's ParticleRenderer.")]
        public bool renderParticles = true;

        public UnifiedWorld World { get; private set; }
        public bool IsBuilt { get; private set; }

        /// <summary>Indices of the particles created by this body.</summary>
        public readonly List<int> Particles = new List<int>();

        /// <summary>
        /// Phase group of this body's particles, for rendering. -1 = the group of the first particle in
        /// <see cref="Particles"/> (override it for bodies that create particles later, like emitters).
        /// </summary>
        public virtual int ParticleGroup => -1;

        protected virtual void OnEnable()
        {
            World = UnifiedWorld.Find(this);
            if (World == null)
            {
                Debug.LogError($"{name}: no UnifiedWorld found in the scene.", this);
                return;
            }
            World.Register(this);
        }

        /// <summary>Creates the body in the solver (once). Call it to force a build order between bodies.</summary>
        public void Build(UnifiedSolver solver)
        {
            if (IsBuilt) return;
            IsBuilt = true;
            Create(solver);
        }

        /// <summary>Creates the particles and constraints.</summary>
        protected abstract void Create(UnifiedSolver solver);

        /// <summary>Called before each simulation frame (e.g. to follow a transform).</summary>
        public virtual void PreStep(UnifiedSolver solver, float h)
        {
        }

        /// <summary>Called once per rendered frame after the simulation, to update meshes or transforms.</summary>
        public virtual void SyncVisual(UnifiedSolver solver)
        {
        }
    }
}
