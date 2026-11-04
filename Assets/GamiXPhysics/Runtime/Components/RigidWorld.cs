using System;
using System.Collections.Generic;
using GamiX.Physics.Collision;
using GamiX.Physics.RigidBodies;
using UnityEngine;

namespace GamiX.Physics.Components
{
    /// <summary>
    /// Scene entry point of the classic rigid body engine (chapter 2 part 2 + chapter 4): Baraff bodies with inertia
    /// tensors, SAT box contacts and an impulse solver. Independent from the particle world; both can coexist.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    [DisallowMultipleComponent]
    public sealed class RigidWorld : MonoBehaviour
    {
        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        public RigidIntegrator integrator = RigidIntegrator.SemiImplicitEuler;
        public BroadPhaseType broadPhase = BroadPhaseType.SweepAndPrune;
        public float stepRate = 60f;
        public int substeps = 2;
        public int solverIterations = 12;
        public bool allowSleeping = true;
        public bool paused;
        public bool showStats = true;

        public RigidBodyWorld World { get; private set; }

        public event Action<RigidBodyWorld, float> BeforeStep;
        public event Action<RigidBodyWorld, float> AfterStep;

        private readonly List<RigidBodyComponent> components = new List<RigidBodyComponent>();
        private float accumulator;
        private double lastMs;
        private BroadPhaseType currentBroadPhase = (BroadPhaseType)(-1);
        private readonly System.Diagnostics.Stopwatch stopwatch = new System.Diagnostics.Stopwatch();

        public static RigidWorld Find(Component from)
        {
            RigidWorld w = from.GetComponentInParent<RigidWorld>();
            return w != null ? w : FindAnyObjectByType<RigidWorld>();
        }

        private void Awake()
        {
            World = new RigidBodyWorld();
            ApplySettings();
        }

        private void ApplySettings()
        {
            World.gravity = gravity;
            World.integrator = integrator;
            World.substeps = substeps;
            World.solverIterations = solverIterations;
            World.allowSleeping = allowSleeping;
            if (currentBroadPhase != broadPhase || World.broadPhase == null)
            {
                World.broadPhase = BroadPhaseFactory.Create(broadPhase);
                currentBroadPhase = broadPhase;
            }
        }

        internal void Register(RigidBodyComponent c)
        {
            if (!components.Contains(c))
                components.Add(c);
        }

        internal void Unregister(RigidBodyComponent c)
        {
            components.Remove(c);
        }

        private void Update()
        {
            if (paused) return;
            ApplySettings();
            float h = 1f / Mathf.Max(1f, stepRate);
            accumulator += Time.deltaTime;
            int steps = 0;
            while (accumulator >= h && steps < 2)
            {
                stopwatch.Restart();
                BeforeStep?.Invoke(World, h);
                World.Step(h);
                AfterStep?.Invoke(World, h);
                lastMs = stopwatch.Elapsed.TotalMilliseconds;
                accumulator -= h;
                steps++;
            }
            if (steps == 2) accumulator = 0f;
            foreach (RigidBodyComponent c in components)
                c.SyncVisual();
        }

        private void OnGUI()
        {
            if (!showStats || World == null) return;
            int sleeping = 0;
            foreach (RigidBody b in World.Bodies) if (b.isSleeping) sleeping++;
            GUI.Label(new Rect(10, 30, 700, 22),
                $"Rigid bodies: {World.Bodies.Count} (sleeping {sleeping})   Pairs: {World.PairCount}   Contacts: {World.Contacts.Count}   " +
                $"Broad phase tests: {World.broadPhase.TestsLastFrame} ({World.broadPhase.Name})   {lastMs:F2} ms");
        }
    }
}
