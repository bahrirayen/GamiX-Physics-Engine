using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>Procedural textures for the cloth showcase (no image assets needed).</summary>
    public static class ClothTextures
    {
        private static Texture2D Make(int w, int h, System.Func<float, float, Color> pixel, TextureWrapMode wrap = TextureWrapMode.Clamp)
        {
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, true) { wrapMode = wrap, filterMode = FilterMode.Trilinear, anisoLevel = 4 };
            var px = new Color[w * h];
            for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                px[y * w + x] = pixel((x + 0.5f) / w, (y + 0.5f) / h);
            tex.SetPixels(px);
            tex.Apply(true);
            return tex;
        }

        private static float Smooth(float edge, float width, float x) => Mathf.Clamp01((x - edge) / width + 0.5f);

        /// <summary>Flag: navy field, lime band and a ring emblem near the hoist (u = 0 at the pole).</summary>
        public static Texture2D Flag()
        {
            Color navy = new Color(0.07f, 0.11f, 0.24f), lime = new Color(0.78f, 1f, 0.1f), white = new Color(0.95f, 0.96f, 0.97f);
            return Make(512, 340, (u, v) =>
            {
                Color c = navy;
                float band = Mathf.Abs(v - 0.5f);
                c = Color.Lerp(c, lime, 1f - Smooth(0.09f, 0.01f, band));
                c = Color.Lerp(c, white, Smooth(0.11f, 0.01f, band) * (1f - Smooth(0.14f, 0.01f, band)));
                float du = (u - 0.24f) * 1.5f, dv = v - 0.5f;
                float r = Mathf.Sqrt(du * du + dv * dv);
                float ring = 1f - Smooth(0.03f, 0.012f, Mathf.Abs(r - 0.25f));
                c = Color.Lerp(c, white, ring);
                c = Color.Lerp(c, navy, 1f - Smooth(0.2f, 0.012f, r));
                c = Color.Lerp(c, lime, 1f - Smooth(0.09f, 0.012f, r));
                return c * (0.92f + 0.08f * Mathf.PerlinNoise(u * 60f, v * 60f));
            });
        }

        /// <summary>Tartan: shows that the UVs stay continuous on each side of a tear.</summary>
        public static Texture2D Tartan()
        {
            Color red = new Color(0.62f, 0.07f, 0.09f), dark = new Color(0.12f, 0.06f, 0.08f), gold = new Color(0.95f, 0.75f, 0.25f);
            return Make(512, 512, (u, v) =>
            {
                float Stripe(float t)
                {
                    float f = t * 6f % 1f;
                    return 1f - Smooth(0.18f, 0.02f, Mathf.Abs(f - 0.5f));
                }
                float Thin(float t)
                {
                    float f = t * 6f % 1f;
                    return 1f - Smooth(0.015f, 0.008f, Mathf.Abs(f - 0.08f));
                }
                Color c = red;
                c = Color.Lerp(c, dark, 0.55f * Stripe(u));
                c = Color.Lerp(c, dark, 0.55f * Stripe(v));
                c = Color.Lerp(c, gold, 0.8f * Mathf.Max(Thin(u), Thin(v)));
                // Woven look: alternating threads.
                float weave = ((int)(u * 256f) + (int)(v * 256f)) % 2 == 0 ? 1f : 0.93f;
                return c * weave;
            });
        }

        /// <summary>Linen sheet with a light blue gingham.</summary>
        public static Texture2D Gingham()
        {
            Color white = new Color(0.94f, 0.95f, 0.96f), blue = new Color(0.45f, 0.62f, 0.9f);
            return Make(512, 512, (u, v) =>
            {
                float a = (u * 10f % 1f) < 0.5f ? 1f : 0f, b = (v * 10f % 1f) < 0.5f ? 1f : 0f;
                float k = 0.28f * a + 0.28f * b;
                return Color.Lerp(white, blue, k) * (0.95f + 0.05f * Mathf.PerlinNoise(u * 80f, v * 80f));
            });
        }

        /// <summary>Checker floor, like the paper's figures (tiled).</summary>
        public static Texture2D Floor()
        {
            Color a = new Color(0.74f, 0.75f, 0.77f), b = new Color(0.66f, 0.67f, 0.7f);
            return Make(128, 128, (u, v) => ((u < 0.5f) ^ (v < 0.5f)) ? a : b, TextureWrapMode.Repeat);
        }
    }
}
