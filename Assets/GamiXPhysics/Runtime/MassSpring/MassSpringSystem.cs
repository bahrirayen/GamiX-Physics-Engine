using System.Collections.Generic;
using GamiX.Physics.Geometry;
using GamiX.Physics.Integration;
using UnityEngine;

namespace GamiX.Physics.MassSpring
{
    /// <summary>A force acting on the particle system. Forces add into <see cref="MassSpringSystem.Forces"/>.</summary>
    public interface IParticleForce
    {
        void Apply(MassSpringSystem system);
    }

    public enum MassSpringIntegrator
    {
        ExplicitEuler,
        SemiImplicitEuler,
        Verlet,
        Midpoint,
        RungeKutta4
    }

    /// <summary>
    /// Force-based particle system of chapter 2 part 1 (slides 29-33). Each particle is (x, v, f, m); the ODE solver
    /// only sees the 6N vector [x1 v1 x2 v2 ...] and its derivative [v1 f1/m1 v2 f2/m2 ...]. The forces are a list of
    /// objects that can be added or removed without changing the particle configuration (slide 33).
    /// </summary>
    public sealed class MassSpringSystem : IOdeSystem
    {
        public readonly List<Vector3> Positions = new List<Vector3>();
        public readonly List<Vector3> Velocities = new List<Vector3>();
        public readonly List<Vector3> Forces = new List<Vector3>();
        public readonly List<float> InverseMasses = new List<float>();
        private readonly List<Vector3> previousPositions = new List<Vector3>();

        /// <summary>The list of force objects to invoke (slide 33).</summary>
        public readonly List<IParticleForce> ForceObjects = new List<IParticleForce>();

        /// <summary>Static planes the particles bounce on (chapter 4, application 1).</summary>
        public readonly List<Plane3D> Planes = new List<Plane3D>();

        public MassSpringIntegrator integrator = MassSpringIntegrator.RungeKutta4;
        public float restitution = 0.5f;
        public float friction = 0.1f;
        public float time;

        private readonly OdeSolver solver = new OdeSolver();

        public int Count => Positions.Count;

        /// <summary>Adds a particle. A mass &lt;= 0 makes it fixed (infinite mass).</summary>
        public int AddParticle(Vector3 position, float mass, Vector3 velocity = default)
        {
            Positions.Add(position);
            Velocities.Add(velocity);
            Forces.Add(VMath.Zero);
            InverseMasses.Add(mass > 0f ? 1f / mass : 0f);
            previousPositions.Add(position);
            return Positions.Count - 1;
        }

        public float Mass(int i) => InverseMasses[i] > 0f ? 1f / InverseMasses[i] : float.PositiveInfinity;

        public void AddForce(int i, Vector3 f)
        {
            Forces[i] += f;
        }

        private void ComputeForces()
        {
            for (int i = 0; i < Forces.Count; i++)
                Forces[i] = VMath.Zero;
            for (int k = 0; k < ForceObjects.Count; k++)
                ForceObjects[k].Apply(this);
        }

        // ---- IOdeSystem: state = [x v] per particle ----

        public int StateSize => 6 * Positions.Count;

        public void GetState(float[] y)
        {
            for (int i = 0, k = 0; i < Positions.Count; i++, k += 6)
            {
                Vector3 x = Positions[i], v = Velocities[i];
                y[k] = x.x; y[k + 1] = x.y; y[k + 2] = x.z;
                y[k + 3] = v.x; y[k + 4] = v.y; y[k + 5] = v.z;
            }
        }

        public void SetState(float[] y)
        {
            for (int i = 0, k = 0; i < Positions.Count; i++, k += 6)
            {
                Positions[i] = new Vector3(y[k], y[k + 1], y[k + 2]);
                Velocities[i] = new Vector3(y[k + 3], y[k + 4], y[k + 5]);
            }
        }

        public void Derivative(float t, float[] y, float[] dydt)
        {
            SetState(y);
            ComputeForces();
            for (int i = 0, k = 0; i < Positions.Count; i++, k += 6)
            {
                Vector3 v = Velocities[i];
                Vector3 a = Forces[i] * InverseMasses[i];
                dydt[k] = v.x; dydt[k + 1] = v.y; dydt[k + 2] = v.z;
                dydt[k + 3] = a.x; dydt[k + 4] = a.y; dydt[k + 5] = a.z;
            }
        }

        /// <summary>Advances the system by dt with the chosen integrator, then handles plane collisions.</summary>
        public void Step(float dt)
        {
            switch (integrator)
            {
                case MassSpringIntegrator.ExplicitEuler:
                    solver.Step(this, time, dt, OdeMethod.ExplicitEuler);
                    break;
                case MassSpringIntegrator.Midpoint:
                    solver.Step(this, time, dt, OdeMethod.Midpoint);
                    break;
                case MassSpringIntegrator.RungeKutta4:
                    solver.Step(this, time, dt, OdeMethod.RungeKutta4);
                    break;
                case MassSpringIntegrator.SemiImplicitEuler:
                    ComputeForces();
                    for (int i = 0; i < Positions.Count; i++)
                    {
                        Velocities[i] += Forces[i] * (InverseMasses[i] * dt);
                        Positions[i] += Velocities[i] * dt;
                    }
                    break;
                case MassSpringIntegrator.Verlet:
                    ComputeForces();
                    for (int i = 0; i < Positions.Count; i++)
                    {
                        Vector3 x = Positions[i];
                        Vector3 next = x + (x - previousPositions[i]) + Forces[i] * (InverseMasses[i] * dt * dt);
                        if (InverseMasses[i] == 0f)
                            next = x;
                        previousPositions[i] = x;
                        Positions[i] = next;
                        Velocities[i] = (next - x) / dt;
                    }
                    break;
            }

            // Fixed particles never move.
            for (int i = 0; i < Positions.Count; i++)
            {
                if (InverseMasses[i] == 0f)
                    Velocities[i] = VMath.Zero;
                if (integrator != MassSpringIntegrator.Verlet)
                    previousPositions[i] = Positions[i] - Velocities[i] * dt;
            }

            ResolvePlaneCollisions();
            time += dt;
        }

        /// <summary>
        /// Contact with static planes (chapter 4, application 1): project back onto the surface,
        /// v_n &lt;- -e v_n and Coulomb-like damping of the tangential velocity.
        /// </summary>
        private void ResolvePlaneCollisions()
        {
            for (int p = 0; p < Planes.Count; p++)
            {
                Plane3D plane = Planes[p];
                for (int i = 0; i < Positions.Count; i++)
                {
                    if (InverseMasses[i] == 0f)
                        continue;
                    float dist = plane.SignedDistance(Positions[i]);
                    if (dist >= 0f)
                        continue;
                    Positions[i] -= plane.normal * dist;
                    Vector3 v = Velocities[i];
                    float vn = VMath.Dot(v, plane.normal);
                    if (vn < 0f)
                    {
                        Vector3 vt = v - plane.normal * vn;
                        v = vt * (1f - friction) - plane.normal * (vn * restitution);
                    }
                    Velocities[i] = v;
                    previousPositions[i] = Positions[i] - v * 0.0001f;
                }
            }
        }

        /// <summary>Total kinetic energy 1/2 sum m v^2 (for checking energy behaviour of each integrator).</summary>
        public float KineticEnergy()
        {
            float e = 0f;
            for (int i = 0; i < Positions.Count; i++)
                if (InverseMasses[i] > 0f)
                    e += 0.5f * VMath.LengthSq(Velocities[i]) / InverseMasses[i];
            return e;
        }
    }

    /// <summary>Weight F = m g on every free particle.</summary>
    public sealed class GravityForce : IParticleForce
    {
        public Vector3 gravity;

        public GravityForce(Vector3 gravity)
        {
            this.gravity = gravity;
        }

        public void Apply(MassSpringSystem s)
        {
            for (int i = 0; i < s.Count; i++)
                if (s.InverseMasses[i] > 0f)
                    s.Forces[i] += gravity / s.InverseMasses[i];
        }
    }

    /// <summary>Viscous friction F = -lambda v (chapter 2, slide 6).</summary>
    public sealed class ViscousDragForce : IParticleForce
    {
        public float lambda;

        public ViscousDragForce(float lambda)
        {
            this.lambda = lambda;
        }

        public void Apply(MassSpringSystem s)
        {
            for (int i = 0; i < s.Count; i++)
                s.Forces[i] -= s.Velocities[i] * lambda;
        }
    }

    /// <summary>
    /// Damped spring between two particles: Hooke F = -k (|d| - L0) d/|d| plus damping -c (v_rel . d/|d|) d/|d|
    /// (chapter 2, slides 19-29).
    /// </summary>
    public sealed class SpringForce : IParticleForce
    {
        public int a, b;
        public float stiffness, damping, restLength;

        public SpringForce(int a, int b, float stiffness, float damping, float restLength)
        {
            this.a = a;
            this.b = b;
            this.stiffness = stiffness;
            this.damping = damping;
            this.restLength = restLength;
        }

        public void Apply(MassSpringSystem s)
        {
            Vector3 d = s.Positions[b] - s.Positions[a];
            float len = VMath.Length(d);
            if (len < VMath.Epsilon)
                return;
            Vector3 dir = d / len;
            float relSpeed = VMath.Dot(s.Velocities[b] - s.Velocities[a], dir);
            Vector3 f = dir * (stiffness * (len - restLength) + damping * relSpeed);
            s.Forces[a] += f;
            s.Forces[b] -= f;
        }
    }

    /// <summary>Spring from a particle to a fixed anchor point (mass-spring-damper of slides 19-21).</summary>
    public sealed class AnchorSpringForce : IParticleForce
    {
        public int particle;
        public Vector3 anchor;
        public float stiffness, damping, restLength;

        public AnchorSpringForce(int particle, Vector3 anchor, float stiffness, float damping, float restLength)
        {
            this.particle = particle;
            this.anchor = anchor;
            this.stiffness = stiffness;
            this.damping = damping;
            this.restLength = restLength;
        }

        public void Apply(MassSpringSystem s)
        {
            Vector3 d = s.Positions[particle] - anchor;
            float len = VMath.Length(d);
            if (len < VMath.Epsilon)
                return;
            Vector3 dir = d / len;
            float speed = VMath.Dot(s.Velocities[particle], dir);
            s.Forces[particle] -= dir * (stiffness * (len - restLength) + damping * speed);
        }
    }

    /// <summary>Uniform wind with a drag coefficient: F = k (v_wind - v).</summary>
    public sealed class WindForce : IParticleForce
    {
        public Vector3 windVelocity;
        public float coefficient;

        public WindForce(Vector3 windVelocity, float coefficient)
        {
            this.windVelocity = windVelocity;
            this.coefficient = coefficient;
        }

        public void Apply(MassSpringSystem s)
        {
            for (int i = 0; i < s.Count; i++)
                s.Forces[i] += (windVelocity - s.Velocities[i]) * coefficient;
        }
    }
}
