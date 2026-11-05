using System;
using GamiX.Physics.Unified;
using UnityEngine;

namespace GamiX.Sim.Yassine
{
    /// <summary>
    /// On-screen control panel of <see cref="FluidLab"/> (IMGUI): scenario buttons, live sliders bound to the solver
    /// parameters, actions, a metric with its recent history, and statistics. Display and input only.
    /// </summary>
    public sealed class FluidControlPanel
    {
        public const float Width = 310f;

        private static readonly Color Accent = new Color(0.8f, 0.95f, 0.2f);
        private static readonly Color Warn = new Color(1f, 0.55f, 0.15f);
        private static readonly Color PanelBg = new Color(0.07f, 0.08f, 0.1f, 0.88f);

        private GUIStyle panel, header, label, value, button, buttonOn, small, caption, captionTitle, richLabel, richLine, richSmall, smallRight;
        private Texture2D white;

        /// <summary>Screen rectangle of the panel last frame, in GUI coordinates (y down).</summary>
        public Rect Rect { get; private set; }

        // ---------------------------------------------------------------- metric history (last 6 s, 10 samples/s)

        public const int HistoryLength = 60;
        private readonly float[] history = new float[HistoryLength];
        private int historyCount, historyHead;

        public void PushSample(float v)
        {
            history[historyHead] = v;
            historyHead = (historyHead + 1) % HistoryLength;
            historyCount = Math.Min(historyCount + 1, HistoryLength);
        }

        public void ClearHistory()
        {
            historyCount = 0;
            historyHead = 0;
        }

        // ---------------------------------------------------------------- styles

        private void EnsureStyles()
        {
            if (panel != null) return;
            white = Texture2D.whiteTexture;
            var bg = new Texture2D(1, 1);
            bg.SetPixel(0, 0, PanelBg);
            bg.Apply();
            panel = new GUIStyle { normal = { background = bg }, padding = new RectOffset(12, 12, 10, 10) };
            header = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = 12, normal = { textColor = Accent }, margin = new RectOffset(0, 0, 8, 2) };
            label = new GUIStyle(GUI.skin.label) { fontSize = 13, normal = { textColor = new Color(0.9f, 0.9f, 0.92f) } };
            value = new GUIStyle(label) { fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleRight, normal = { textColor = Color.white } };
            small = new GUIStyle(label) { fontSize = 10, normal = { textColor = new Color(0.6f, 0.62f, 0.66f) } };
            button = new GUIStyle(GUI.skin.button) { fontSize = 12, fixedHeight = 24 };
            buttonOn = new GUIStyle(button) { fontStyle = FontStyle.Bold, normal = { textColor = Accent }, hover = { textColor = Accent } };
            caption = new GUIStyle(label) { fontSize = 12, wordWrap = true, normal = { textColor = new Color(0.85f, 0.86f, 0.9f) } };
            captionTitle = new GUIStyle(label) { fontSize = 18, fontStyle = FontStyle.Bold, normal = { textColor = Color.white } };
            richLabel = new GUIStyle(label) { richText = true };
            richLine = new GUIStyle(label) { richText = true, fontSize = 12 };
            richSmall = new GUIStyle(small) { richText = true };
            smallRight = new GUIStyle(small) { alignment = TextAnchor.LowerRight };
        }

        // ---------------------------------------------------------------- drawing

        public void Draw(FluidLab lab)
        {
            EnsureStyles();
            UnifiedSolver s = lab.World != null ? lab.World.Solver : null;
            if (s == null || lab.Setup == null) return;
            SolverParams p = s.parameters;
            ScenarioSetup setup = lab.Setup;

            DrawCaption(setup);

            GUILayout.BeginArea(new Rect(Screen.width - Width - 10f, 10f, Width, Screen.height - 20f));
            GUILayout.BeginVertical(panel);

            Header("SCENARIO");
            GUILayout.BeginHorizontal();
            ScenarioButton(lab, FluidScenario.DamBreak, "Dam break");
            ScenarioButton(lab, FluidScenario.Buoyancy, "Buoyancy");
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            ScenarioButton(lab, FluidScenario.RayleighTaylor, "Rayleigh-Taylor");
            ScenarioButton(lab, FluidScenario.Fountain, "Fountain");
            GUILayout.EndHorizontal();

            Header("FLUID");
            p.viscosity = Slider("Viscosity", p.viscosity, 0f, 0.2f, "F3");
            p.cohesion = Slider("Cohesion", p.cohesion, 0f, 0.3f, "F2");
            p.surfaceTension = Slider("Surface tension", p.surfaceTension, 0f, 0.3f, "F2");
            p.vorticityConfinement = Slider("Vorticity  ε", p.vorticityConfinement, 0f, 3f, "F2");
            p.solidDensityScale = Slider("Solid weight  s", p.solidDensityScale, 0f, 1f, "F2");

            if (setup.emitter != null)
            {
                Header("EMITTER");
                setup.emitter.speed = Slider("Jet speed", setup.emitter.speed, 0.5f, 5f, "F1", " m/s");
            }

            Header("SOLVER");
            p.substeps = Mathf.RoundToInt(Slider("Sub-steps", p.substeps, 1f, 6f, "F0"));
            p.iterations = Mathf.RoundToInt(Slider("Iterations", p.iterations, 1f, 10f, "F0"));
            p.relaxation = Slider("Relaxation  ω", p.relaxation, 0.5f, 2f, "F2");
            float g = Slider("Gravity", -p.gravity.y, 0f, 20f, "F1", " m/s²");
            p.gravity = new Vector3(0f, -g, 0f);

            GUILayout.Space(8f);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(lab.ShowSurface ? "Surface" : "Particles", button)) lab.ShowSurface = !lab.ShowSurface;
            if (GUILayout.Button("Colors: " + lab.colorMode, lab.ShowSurface ? button : buttonOn)) lab.CycleColors();
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Drop crate", button)) lab.DropCrate();
            if (GUILayout.Button("Splash", button)) lab.Splash();
            if (GUILayout.Button(lab.World.paused ? "Play" : "Pause", button)) lab.World.paused = !lab.World.paused;
            if (GUILayout.Button("Reset", button)) lab.RequestBuild();
            GUILayout.EndHorizontal();
            if (setup.emitter != null && GUILayout.Button(setup.emitter.emitting ? "Emitter on" : "Emitter off", setup.emitter.emitting ? buttonOn : button))
                setup.emitter.emitting = !setup.emitter.emitting;

            GUILayout.Space(10f);
            Row("Max compression", $"{100f * lab.MaxCompression:F1} %");
            DrawHistory(lab.CompressionReference);
            GUILayout.BeginHorizontal();
            GUILayout.Label($"<color=#CCF233>■</color> under {100f * lab.CompressionReference:0} %   <color=#FF8C26>■</color> over   last 6 s",
                            richSmall);
            GUILayout.EndHorizontal();

            GUILayout.Space(4f);
            foreach (string line in lab.ScenarioLines)
                GUILayout.Label(line, richLine);
            GUILayout.Label($"<b>{s.ActiveCount:N0}</b> particles  ·  <b>{s.LastStepMilliseconds:F1}</b> ms  ·  <b>{lab.Fps:F0}</b> FPS",
                            richLabel);
            GUILayout.Label("1-4 scenario · R reset · V view · C colors · B crate · H hide", small);

            GUILayout.EndVertical();
            Rect r = GUILayoutUtility.GetLastRect();
            GUILayout.EndArea();
            if (Event.current.type == EventType.Repaint)
                Rect = new Rect(r.x + Screen.width - Width - 10f, r.y + 10f, r.width, r.height);
        }

        private void DrawCaption(ScenarioSetup setup)
        {
            var area = new Rect(14f, 10f, Mathf.Min(460f, Screen.width - Width - 40f), 120f);
            GUI.Label(new Rect(area.x, area.y, area.width, 26f), setup.title, captionTitle);
            GUI.Label(new Rect(area.x, area.y + 28f, area.width, 90f), setup.description, caption);
        }

        private void Header(string text)
        {
            GUILayout.Label(text, header);
        }

        private void ScenarioButton(FluidLab lab, FluidScenario sc, string text)
        {
            if (GUILayout.Button(text, lab.scenario == sc ? buttonOn : button) && lab.scenario != sc)
            {
                lab.scenario = sc;
                lab.RequestBuild();
            }
        }

        private float Slider(string name, float v, float min, float max, string format, string unit = "")
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(name, label, GUILayout.Width(118f));
            GUILayout.BeginVertical();
            GUILayout.Space(8f);
            v = GUILayout.HorizontalSlider(v, min, max, GUILayout.Width(88f));
            GUILayout.EndVertical();
            GUILayout.Label(v.ToString(format) + unit, value);
            GUILayout.EndHorizontal();
            return v;
        }

        private void Row(string name, string v)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(name, label);
            GUILayout.Label(v, value);
            GUILayout.EndHorizontal();
        }

        /// <summary>Bar chart of the metric history, with a dashed reference line at the threshold.</summary>
        private void DrawHistory(float reference)
        {
            Rect r = GUILayoutUtility.GetRect(Width - 24f, 34f);
            if (Event.current.type != EventType.Repaint) return;
            float top = Mathf.Max(reference * 2f, 1e-4f);
            Color old = GUI.color;
            GUI.color = new Color(1f, 1f, 1f, 0.06f);
            GUI.DrawTexture(r, white);
            float barW = r.width / HistoryLength;
            for (int k = 0; k < historyCount; k++)
            {
                float v = history[(historyHead - historyCount + k + HistoryLength) % HistoryLength];
                float h = Mathf.Clamp01(v / top) * r.height;
                GUI.color = v > reference ? Warn : Accent;
                GUI.DrawTexture(new Rect(r.x + k * barW + 0.5f, r.yMax - h, Mathf.Max(1f, barW - 1f), Mathf.Max(1f, h)), white);
            }
            GUI.color = new Color(1f, 1f, 1f, 0.45f);
            float y = r.yMax - r.height * reference / top;
            for (float x = r.x; x < r.xMax; x += 6f)
                GUI.DrawTexture(new Rect(x, y, 3f, 1f), white);
            GUI.color = old;
            GUI.Label(new Rect(r.xMax - 40f, y - 14f, 40f, 14f), $"{100f * reference:0} %", smallRight);
        }
    }
}
