using System;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>
    /// Particle phase (paper, section 3): an integer that groups particles and controls how they interact.
    /// Low 24 bits = group id, high bits = flags.
    /// </summary>
    public static class Phase
    {
        public const int GroupMask = 0x00FFFFFF;

        /// <summary>Particles of the same group collide with each other (cloth, granular, ropes).</summary>
        public const int SelfCollide = 1 << 24;

        /// <summary>Fluid particle: interacts with other fluid particles through the density constraint.</summary>
        public const int Fluid = 1 << 25;

        /// <summary>With SelfCollide: ignore pairs that are closer than the contact distance in the rest pose
        /// (neighbours in a cloth would otherwise push each other).</summary>
        public const int SelfCollideFilter = 1 << 26;

        public static int Make(int group, int flags = 0) => (group & GroupMask) | flags;
        public static int Group(int phase) => phase & GroupMask;
        public static bool IsFluid(int phase) => (phase & Fluid) != 0;
    }

    /// <summary>All the user parameters of the unified solver.</summary>
    [Serializable]
    public sealed class SolverParams
    {
        [Header("Time stepping (Algorithm 1)")]
        [Tooltip("Sub-steps per frame.")]
        public int substeps = 2;
        [Tooltip("Constraint iterations per sub-step.")]
        public int iterations = 4;
        [Tooltip("Pre-stabilization iterations on contacts (section 4.4). 0 disables it.")]
        public int stabilizationIterations = 1;
        [Tooltip("Successive over-relaxation factor omega of eq. 13 (1 <= omega <= 2).")]
        [Range(0.5f, 2f)] public float relaxation = 1f;

        [Header("Particles")]
        [Tooltip("Particle radius r. Solid particles touch at a distance of 2r.")]
        public float radius = 0.05f;
        [Tooltip("Extra search distance for neighbours and contacts, as a fraction of the radius (section 9).")]
        public float collisionMargin = 0.2f;
        [Tooltip("Max neighbours stored per particle.")]
        public int maxNeighbors = 96;

        [Header("Forces")]
        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        [Tooltip("Global velocity damping per second.")]
        public float damping = 0f;
        [Tooltip("Velocities are clamped to this magnitude (stability safety net).")]
        public float maxSpeed = 100f;
        [Tooltip("Environment velocity (wind), read by extensions such as aerodynamics or gas drag.")]
        public Vector3 wind = Vector3.zero;

        [Header("Friction (section 6.1)")]
        [Tooltip("Static friction coefficient mu_s: tangential motion below mu_s * d is removed.")]
        public float staticFriction = 0.4f;
        [Tooltip("Kinetic friction coefficient mu_k: tangential motion is limited by mu_k * d.")]
        public float dynamicFriction = 0.3f;
        [Tooltip("Friction multiplier for particle-particle contacts.")]
        public float particleFriction = 1f;
        [Tooltip("Friction multiplier for contacts involving fluid particles (fluids should not stick to walls).")]
        public float fluidFriction = 0f;
        [Tooltip("Restitution against kinematic shapes (0 = no bounce).")]
        public float shapeRestitution = 0f;

        [Header("Rigid stacks (section 5.2) and sleeping (section 4.5)")]
        [Tooltip("Shock propagation k: contact mass scaling s = exp(-k h). 0 disables it.")]
        public float shockPropagation = 0f;
        [Tooltip("Height of the reference ground for the stack height h.")]
        public float groundHeight = 0f;
        [Tooltip("Particle sleeping threshold epsilon (eq. 14), in meters per sub-step. 0 disables it.")]
        public float sleepThreshold = 0f;

        [Header("Fluids (section 7)")]
        [Tooltip("Rest spacing of fluid particles, as a multiple of 2r.")]
        public float fluidRestSpacing = 1f;
        [Tooltip("SPH kernel radius h, as a multiple of the fluid rest spacing.")]
        public float kernelScale = 2f;
        [Tooltip("Constraint relaxation added to the density denominator (fraction of the reference value).")]
        public float fluidRelaxation = 0.01f;
        [Tooltip("Unilateral density constraint (eq. 26): only separate particles.")]
        public bool unilateralDensity = true;
        [Tooltip("Weight s of solid particles in the fluid density (eq. 27). 0.5 gives realistic floating depths.")]
        public float solidDensityScale = 0.5f;
        [Tooltip("XSPH viscosity.")]
        public float viscosity = 0.01f;
        [Tooltip("Vorticity confinement strength.")]
        public float vorticityConfinement = 0f;
        [Tooltip("Akinci cohesion, as an acceleration in units of g per unit of kernel weight.")]
        public float cohesion = 0.05f;
        [Tooltip("Akinci surface tension, as an acceleration in units of g.")]
        public float surfaceTension = 0f;
        [Tooltip("Gravity multiplier for fluid particles (< 0 makes a fluid rise, e.g. hot gas).")]
        public float fluidGravityScale = 1f;

        [Header("Constraint solving")]
        [Tooltip("Solve distance constraints (cloth, ropes) in Gauss-Seidel order instead of Jacobi. " +
                 "The paper uses Jacobi for GPU parallelism; on the CPU Gauss-Seidel converges much faster.")]
        public bool gaussSeidelDistance = true;

        [Header("Performance")]
        [Tooltip("Run particle-centric loops on several threads (Jacobi iterations allow it).")]
        public bool multithreaded = true;

        public float SolidRestDistance => 2f * radius;
        public float FluidRestDistance => 2f * radius * fluidRestSpacing;
        public float KernelRadius => FluidRestDistance * kernelScale;
    }
}
