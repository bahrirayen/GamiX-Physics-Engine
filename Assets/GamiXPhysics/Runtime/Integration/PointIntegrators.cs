using UnityEngine;

namespace GamiX.Physics.Integration
{
    /// <summary>Acceleration of a point as a function of its position, velocity and time: a = F(x, v, t) / m.</summary>
    public delegate Vector3 AccelerationFunc(Vector3 position, Vector3 velocity, float time);

    public enum PointIntegrator
    {
        ExplicitEuler,
        SemiImplicitEuler,
        Verlet,
        RungeKutta4
    }

    /// <summary>
    /// Integrators for a single material point (chapter 2 part 1). The second order equation x'' = a(x, v, t) is
    /// written as the first order system dx/dt = v, dv/dt = a (slide 21).
    /// </summary>
    public static class PointIntegrators
    {
        /// <summary>Explicit Euler: v(t+dt) = v + a dt, x(t+dt) = x + v(t) dt (slide 12).</summary>
        public static void ExplicitEuler(ref Vector3 x, ref Vector3 v, AccelerationFunc acc, float t, float dt)
        {
            Vector3 a = acc(x, v, t);
            x += v * dt;
            v += a * dt;
        }

        /// <summary>Semi-implicit (symplectic) Euler: velocity first, then position with the new velocity.</summary>
        public static void SemiImplicitEuler(ref Vector3 x, ref Vector3 v, AccelerationFunc acc, float t, float dt)
        {
            v += acc(x, v, t) * dt;
            x += v * dt;
        }

        /// <summary>
        /// Position Verlet: x(t+dt) = 2 x(t) - x(t-dt) + a dt^2. The velocity is recovered by finite difference.
        /// <paramref name="previous"/> holds x(t - dt) and is updated.
        /// </summary>
        public static void Verlet(ref Vector3 x, ref Vector3 previous, ref Vector3 v, AccelerationFunc acc, float t, float dt)
        {
            Vector3 a = acc(x, v, t);
            Vector3 next = 2f * x - previous + a * (dt * dt);
            v = (next - x) / dt;
            previous = x;
            x = next;
        }

        /// <summary>
        /// Runge-Kutta 4 on (x, v) for x'' = a(x, v, t) (slide 21): k = increments of x, l = increments of v,
        /// with the acceleration re-evaluated at every stage.
        /// </summary>
        public static void RungeKutta4(ref Vector3 x, ref Vector3 v, AccelerationFunc acc, float t, float dt)
        {
            float h = dt;
            Vector3 k1 = v * h;
            Vector3 l1 = acc(x, v, t) * h;

            Vector3 k2 = (v + l1 * 0.5f) * h;
            Vector3 l2 = acc(x + k1 * 0.5f, v + l1 * 0.5f, t + 0.5f * h) * h;

            Vector3 k3 = (v + l2 * 0.5f) * h;
            Vector3 l3 = acc(x + k2 * 0.5f, v + l2 * 0.5f, t + 0.5f * h) * h;

            Vector3 k4 = (v + l3) * h;
            Vector3 l4 = acc(x + k3, v + l3, t + h) * h;

            x += (k1 + 2f * k2 + 2f * k3 + k4) / 6f;
            v += (l1 + 2f * l2 + 2f * l3 + l4) / 6f;
        }

        /// <summary>Dispatches to one of the integrators (Verlet needs the previous position).</summary>
        public static void Step(PointIntegrator method, ref Vector3 x, ref Vector3 v, ref Vector3 previous,
                                AccelerationFunc acc, float t, float dt)
        {
            switch (method)
            {
                case PointIntegrator.ExplicitEuler: ExplicitEuler(ref x, ref v, acc, t, dt); previous = x - v * dt; break;
                case PointIntegrator.SemiImplicitEuler: SemiImplicitEuler(ref x, ref v, acc, t, dt); previous = x - v * dt; break;
                case PointIntegrator.Verlet: Verlet(ref x, ref previous, ref v, acc, t, dt); break;
                default: RungeKutta4(ref x, ref v, acc, t, dt); previous = x - v * dt; break;
            }
        }

        /// <summary>
        /// Analytic solution of the free fall with viscous friction F = -lambda v (chapter 2, slide 6),
        /// v(t) = v_lim + (v0 - v_lim) e^(-t/tau) with tau = m/lambda. Useful to measure integrator error.
        /// </summary>
        public static float FreeFallWithDragVelocity(float v0, float g, float tau, float t)
        {
            float vLim = g * tau;
            return vLim + (v0 - vLim) * Mathf.Exp(-t / tau);
        }
    }
}
