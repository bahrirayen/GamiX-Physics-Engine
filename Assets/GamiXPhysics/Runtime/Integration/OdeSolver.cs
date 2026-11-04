using System;

namespace GamiX.Physics.Integration
{
    /// <summary>
    /// A system dY/dt = f(Y, t) whose state is a flat vector (chapter 2 part 1, slides 30-33).
    /// The solver never needs to know what the numbers mean: particles, springs or rigid bodies all fit.
    /// </summary>
    public interface IOdeSystem
    {
        int StateSize { get; }

        /// <summary>Copies the current state into y.</summary>
        void GetState(float[] y);

        /// <summary>Overwrites the current state with y.</summary>
        void SetState(float[] y);

        /// <summary>Writes dY/dt evaluated at state y and time t into dydt.</summary>
        void Derivative(float t, float[] y, float[] dydt);
    }

    public enum OdeMethod
    {
        /// <summary>y(t+h) = y(t) + h f(y, t). First order (slide 9).</summary>
        ExplicitEuler,
        /// <summary>Second order: evaluates the slope at the middle of the step.</summary>
        Midpoint,
        /// <summary>Runge-Kutta 4 (slide 14): y(t+h) = y(t) + h/6 (k1 + 2 k2 + 2 k3 + k4).</summary>
        RungeKutta4
    }

    /// <summary>
    /// Generic ODE solver. Buffers are allocated once and reused, so stepping does not allocate.
    /// Unlike a "RK4" that keeps the acceleration constant during the step, every k_i here re-evaluates the
    /// derivative (forces included) at the intermediate state, which is what gives RK4 its 4th order accuracy.
    /// </summary>
    public sealed class OdeSolver
    {
        private float[] y, y0, tmp, k1, k2, k3, k4;

        private void Ensure(int n)
        {
            if (y != null && y.Length == n)
                return;
            y = new float[n];
            y0 = new float[n];
            tmp = new float[n];
            k1 = new float[n];
            k2 = new float[n];
            k3 = new float[n];
            k4 = new float[n];
        }

        public void Step(IOdeSystem system, float t, float h, OdeMethod method)
        {
            int n = system.StateSize;
            Ensure(n);
            system.GetState(y0);

            switch (method)
            {
                case OdeMethod.ExplicitEuler:
                    system.Derivative(t, y0, k1);
                    for (int i = 0; i < n; i++)
                        y[i] = y0[i] + h * k1[i];
                    break;

                case OdeMethod.Midpoint:
                    system.Derivative(t, y0, k1);
                    for (int i = 0; i < n; i++)
                        tmp[i] = y0[i] + 0.5f * h * k1[i];
                    system.Derivative(t + 0.5f * h, tmp, k2);
                    for (int i = 0; i < n; i++)
                        y[i] = y0[i] + h * k2[i];
                    break;

                default:
                    // k1 = f(y, t)
                    system.Derivative(t, y0, k1);
                    // k2 = f(y + h/2 k1, t + h/2)
                    for (int i = 0; i < n; i++)
                        tmp[i] = y0[i] + 0.5f * h * k1[i];
                    system.Derivative(t + 0.5f * h, tmp, k2);
                    // k3 = f(y + h/2 k2, t + h/2)
                    for (int i = 0; i < n; i++)
                        tmp[i] = y0[i] + 0.5f * h * k2[i];
                    system.Derivative(t + 0.5f * h, tmp, k3);
                    // k4 = f(y + h k3, t + h)
                    for (int i = 0; i < n; i++)
                        tmp[i] = y0[i] + h * k3[i];
                    system.Derivative(t + h, tmp, k4);
                    float h6 = h / 6f;
                    for (int i = 0; i < n; i++)
                        y[i] = y0[i] + h6 * (k1[i] + 2f * k2[i] + 2f * k3[i] + k4[i]);
                    break;
            }

            system.SetState(y);
        }
    }

    /// <summary>
    /// Scalar ODE helpers matching the course examples (dy/dt = f(y, t), e.g. dy/dt = -2 y t, y(0) = 1, slide 14).
    /// </summary>
    public static class ScalarOde
    {
        public static float EulerStep(Func<float, float, float> f, float y, float t, float h)
        {
            return y + h * f(y, t);
        }

        public static float Rk4Step(Func<float, float, float> f, float y, float t, float h)
        {
            float k1 = f(y, t);
            float k2 = f(y + 0.5f * h * k1, t + 0.5f * h);
            float k3 = f(y + 0.5f * h * k2, t + 0.5f * h);
            float k4 = f(y + h * k3, t + h);
            return y + h / 6f * (k1 + 2f * k2 + 2f * k3 + k4);
        }
    }
}
