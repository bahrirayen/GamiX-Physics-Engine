using GamiX.Physics.Rendering;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    /// <summary>Display materials of the fluid scene: glossy water, transparent glass, checker floor (no physics here).</summary>
    public static class FluidMaterials
    {
        public static Material Glossy(Color color, float smoothness)
        {
            Material m = RenderUtils.CreateLitMaterial(color);
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", smoothness);
            if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", smoothness);
            return m;
        }

        /// <summary>URP Lit switched to the transparent surface type (alpha blending, no depth write).</summary>
        public static Material Transparent(Color color, float smoothness)
        {
            Material m = Glossy(color, smoothness);
            if (m.HasProperty("_Surface"))
            {
                m.SetFloat("_Surface", 1f);
                m.SetFloat("_Blend", 0f);
                m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
                m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                m.SetFloat("_ZWrite", 0f);
                m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                m.SetOverrideTag("RenderType", "Transparent");
                m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            }
            return m;
        }

        /// <summary>Light grey checker floor, one square per meter / tiles.</summary>
        public static Material Checker(Color a, Color b, float tiles)
        {
            var tex = new Texture2D(2, 2, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Repeat
            };
            tex.SetPixels(new[] { a, b, b, a });
            tex.Apply();
            Material m = Glossy(Color.white, 0.2f);
            if (m.HasProperty("_BaseMap")) m.SetTexture("_BaseMap", tex);
            m.mainTexture = tex;
            m.mainTextureScale = new Vector2(tiles * 0.5f, tiles * 0.5f);
            if (m.HasProperty("_BaseMap")) m.SetTextureScale("_BaseMap", m.mainTextureScale);
            return m;
        }
    }
}
