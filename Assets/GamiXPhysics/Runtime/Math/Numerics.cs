using System;
using UnityEngine;

namespace GamiX.Physics
{
    /// <summary>Root finding used by ray casting (chapter 3, slides 30-32) and collision timing (chapter 4).</summary>
    public static class Numerics
    {
        /// <summary>
        /// Real roots of a t^2 + b t + c = 0, sorted ascending. Returns the number of roots (0, 1 or 2).
        /// Degenerates to the linear case when a is (almost) zero.
        /// </summary>
        public static int SolveQuadratic(float a, float b, float c, out float t0, out float t1)
        {
            t0 = t1 = 0f;
            if (Mathf.Abs(a) < 1e-12f)
            {
                if (Mathf.Abs(b) < 1e-12f)
                    return 0;
                t0 = t1 = -c / b;
                return 1;
            }
            float delta = b * b - 4f * a * c;
            if (delta < 0f)
                return 0;
            if (delta == 0f)
            {
                t0 = t1 = -b / (2f * a);
                return 1;
            }
            // Numerically stable form: avoids cancellation when b^2 >> 4ac.
            float sq = Mathf.Sqrt(delta);
            float q = -0.5f * (b + (b >= 0f ? sq : -sq));
            float r0 = q / a;
            float r1 = c / q;
            t0 = r0 < r1 ? r0 : r1;
            t1 = r0 < r1 ? r1 : r0;
            return 2;
        }

        /// <summary>
        /// Real roots of t^3 + a t^2 + b t + c = 0 (slides 31-32). Substitution t = x - a/3 gives the depressed
        /// cubic x^3 + p x + q = 0, solved with Cardano's formula (one real root) or the trigonometric form
        /// (three real roots). Returns the number of distinct real roots written in <paramref name="roots"/>.
        /// </summary>
        public static int SolveCubic(float a, float b, float c, float[] roots)
        {
            double A = a, B = b, C = c;
            double p = B - A * A / 3.0;
            double q = 2.0 * A * A * A / 27.0 - A * B / 3.0 + C;
            double shift = -A / 3.0;

            // Slide 32: Delta' = Delta / 108 = -((p/3)^3 + (q/2)^2).
            double pp = p / 3.0, qq = q / 2.0;
            double d = -(pp * pp * pp + qq * qq);

            if (Math.Abs(d) < 1e-14)
            {
                // Double root: x1 = 2r, x2 = x3 = -r with r = cbrt(-q/2).
                double r = Cbrt(-qq);
                if (Math.Abs(r) < 1e-14)
                {
                    roots[0] = (float)shift;
                    return 1;
                }
                double x1 = 2.0 * r + shift, x2 = -r + shift;
                roots[0] = (float)Math.Min(x1, x2);
                roots[1] = (float)Math.Max(x1, x2);
                return 2;
            }
            if (d < 0.0)
            {
                // One real root: x = r + s, r = cbrt(-q' + sqrt(-D')), s = cbrt(-q' - sqrt(-D')).
                double sq = Math.Sqrt(-d);
                roots[0] = (float)(Cbrt(-qq + sq) + Cbrt(-qq - sq) + shift);
                return 1;
            }
            // Three distinct real roots (the slide's complex cube roots, written with cosines).
            double m = 2.0 * Math.Sqrt(-pp);
            double theta = Math.Acos(Math.Max(-1.0, Math.Min(1.0, 3.0 * q / (p * m)))) / 3.0;
            double y0 = m * Math.Cos(theta) + shift;
            double y1 = m * Math.Cos(theta - 2.0 * Math.PI / 3.0) + shift;
            double y2 = m * Math.Cos(theta - 4.0 * Math.PI / 3.0) + shift;
            // Sort ascending.
            if (y0 > y1) { double t = y0; y0 = y1; y1 = t; }
            if (y1 > y2) { double t = y1; y1 = y2; y2 = t; }
            if (y0 > y1) { double t = y0; y0 = y1; y1 = t; }
            roots[0] = (float)y0; roots[1] = (float)y1; roots[2] = (float)y2;
            return 3;
        }

        private static double Cbrt(double v)
        {
            return v < 0.0 ? -Math.Pow(-v, 1.0 / 3.0) : Math.Pow(v, 1.0 / 3.0);
        }

        /// <summary>
        /// Bisection / dichotomy (chapter 4, slide 12). Requires f(a) * f(b) &lt; 0 and returns x with f(x) ~ 0.
        /// </summary>
        public static float Bisection(Func<float, float> f, float a, float b, float epsilon = 1e-5f, int maxIterations = 64)
        {
            float fa = f(a);
            for (int i = 0; i < maxIterations && Mathf.Abs(b - a) > epsilon; i++)
            {
                float mid = 0.5f * (a + b);
                float fm = f(mid);
                if (fm * fa < 0f)
                {
                    b = mid;
                }
                else
                {
                    a = mid;
                    fa = fm;
                }
            }
            return 0.5f * (a + b);
        }

        /// <summary>Solves the 2x2 system [a b; c d] [x; y] = [e; f]. Returns false when singular.</summary>
        public static bool Solve2x2(float a, float b, float c, float d, float e, float f, out float x, out float y)
        {
            float det = a * d - b * c;
            if (Mathf.Abs(det) < 1e-12f)
            {
                x = y = 0f;
                return false;
            }
            float inv = 1f / det;
            x = (d * e - b * f) * inv;
            y = (a * f - c * e) * inv;
            return true;
        }
    }
}
