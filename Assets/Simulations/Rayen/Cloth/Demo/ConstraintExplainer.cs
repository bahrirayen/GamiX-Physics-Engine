using System.Collections;
using System.Collections.Generic;
using System.IO;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Slow-motion explainer of position based dynamics (paper, section 4), for the "Constraints" slide.
    /// Every correction shown on screen is computed by our UnifiedSolver:
    ///   1. a distance constraint C = |xi - xj| - L, each particle moves by its share wi / (wi + wj),
    ///   2. a pinned particle (w = 0) never moves, the other one takes the whole correction,
    ///   3. two identical rules on one particle (paper, figure 2): summed corrections overshoot forever,
    ///      averaged corrections x* += omega / n * sum dx settle,
    ///   4. a hanging chain solved every frame by the real solver (Jacobi, averaging, over-relaxation).
    /// Controls: Space = pause, R = restart, 1-4 = jump to a part.
    /// With <see cref="record"/> on, the timeline runs at a fixed frame rate and every frame is saved as a JPG.
    /// </summary>
    public sealed class ConstraintExplainer : MonoBehaviour
    {
        [Header("Recording")]
        public bool record;
        [Tooltip("Folder for the frames (absolute path).")]
        public string recordFolder = "";
        public int recordWidth = 1920;
        public int recordHeight = 1080;
        public int recordFps = 30;

        [Header("Formulas (rendered math, Demo/Formulas)")]
        public Texture2D summedFormula;
        public Texture2D averagedFormula;
        public Texture2D velocityFormula;
        public Texture2D shareHalfFormula;
        public Texture2D shareOneFormula;

        [Header("Playback")]
        public bool loop = true;

        // Palette of the presentation.
        private static readonly Color Background = Hex(0xEFE4DA);
        private static readonly Color Ink = Hex(0x3B2A22);
        private static readonly Color Soft = Hex(0x9A816B);
        private static readonly Color Link = Hex(0x7A604E);
        private static readonly Color Teal = Hex(0x0E93A3);
        private static readonly Color Red = Hex(0xB02A2A);
        private static readonly Color Pinned = Hex(0x5A4A40);
        private static readonly Color Good = Hex(0x2E8B57);

        private static readonly float[] Durations = { 9.5f, 8.5f, 12.5f, 11f };
        private const float Fade = 0.35f;
        private const float BallSize = 0.6f;
        private const float ViewHeight = 4.988f;   // world height visible at z = 0 (camera 10 m away, 28° field of view)

        private Camera cam;
        private Canvas canvas;
        private Font font;
        private Material lineMaterial;
        private Mesh sphere;
        private Image fadeOverlay;
        private readonly List<LineRenderer> lines = new List<LineRenderer>();
        private readonly List<Text> texts = new List<Text>();
        private readonly List<RawImage> images = new List<RawImage>();
        private int linesUsed, textsUsed, imagesUsed;

        private float time;
        private int part = -1;
        private bool paused;

        // Precomputed solver results (positions before and after each projection).
        private Vector3[][] distanceFrames, pinnedFrames, averagedFrames;
        private Vector3[] summedPositions;
        private const float RowTop = 0.75f, RowBottom = -0.95f, AnchorX = 2.6f, TwoRuleRest = 2f;
        private const int TwoRuleIterations = 5;

        // Part 4: live chain.
        private UnifiedSolver chain;
        private int chainSteps;
        private const int ChainCount = 12;
        private const float ChainSpacing = 0.2f;
        private static readonly Vector3 ChainAnchor = new Vector3(-0.8f, 1.0f, 0f);

        // Recording state.
        private RenderTexture target;
        private Texture2D readback;
        private int frame;
        private bool recording;

        private void Awake()
        {
            cam = Camera.main;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Background;
            cam.fieldOfView = 28f;
            cam.nearClipPlane = 0.1f;
            cam.transform.SetPositionAndRotation(new Vector3(0f, 0f, -10f), Quaternion.identity);

            var shader = Shader.Find("Sprites/Default");
            lineMaterial = new Material(shader);
            BuildSphere();
            BuildCanvas();
            Precompute();
        }

        private void Start()
        {
            if (record)
                StartRecording();
            SetPart(0);
        }

        // ================================================================ solver runs

        /// <summary>
        /// Runs our solver on a few particles with gravity off, one sub-step and one Jacobi iteration per step, and
        /// returns the positions before and after each step. Velocities are cleared between steps so every step
        /// shows exactly one projection of the constraints.
        /// </summary>
        private static Vector3[][] Project(Vector3[] start, float[] invMass, (int i, int j, float rest)[] rules, int steps)
        {
            var p = new SolverParams
            {
                substeps = 1, iterations = 1, stabilizationIterations = 0, relaxation = 1f,
                gravity = Vector3.zero, radius = 0.01f, gaussSeidelDistance = false, multithreaded = false
            };
            var solver = new UnifiedSolver(p);
            int phase = Phase.Make(solver.NewGroup());
            for (int k = 0; k < start.Length; k++)
                solver.AddParticle(start[k], Vector3.zero, invMass[k], phase);
            foreach (var r in rules)
            {
                solver.AddDistanceConstraint(r.i, r.j);
                solver.DistanceConstraints.items[solver.DistanceConstraints.Count - 1].restLength = r.rest;
            }

            var frames = new Vector3[steps + 1][];
            frames[0] = (Vector3[])start.Clone();
            for (int s = 1; s <= steps; s++)
            {
                solver.Step(1f / 60f);
                frames[s] = new Vector3[start.Length];
                for (int k = 0; k < start.Length; k++)
                {
                    frames[s][k] = solver.positions[k];
                    solver.velocities[k] = Vector3.zero;
                }
            }
            return frames;
        }

        private void Precompute()
        {
            distanceFrames = Project(new[] { new Vector3(-2f, 0.3f, 0f), new Vector3(2f, 0.3f, 0f) },
                new[] { 1f, 1f }, new[] { (0, 1, 2.6f) }, 1);
            pinnedFrames = Project(new[] { new Vector3(-2f, 0.3f, 0f), new Vector3(1.6f, 0.3f, 0f) },
                new[] { 1f, 0f }, new[] { (0, 1, 2.2f) }, 1);
            averagedFrames = Project(new[] { new Vector3(-0.2f, RowBottom, 0f), new Vector3(AnchorX, RowBottom, 0f) },
                new[] { 1f, 0f }, new[] { (0, 1, TwoRuleRest), (0, 1, TwoRuleRest) }, TwoRuleIterations);

            // Same two corrections, summed instead of averaged (what Jacobi does without the 1/n of eq. 13).
            summedPositions = new Vector3[TwoRuleIterations + 1];
            summedPositions[0] = new Vector3(-0.2f, RowTop, 0f);
            Vector3 anchor = new Vector3(AnchorX, RowTop, 0f);
            for (int s = 1; s <= TwoRuleIterations; s++)
            {
                Vector3 x = summedPositions[s - 1];
                summedPositions[s] = x + 2f * RuleCorrection(x, anchor, TwoRuleRest);
            }
        }

        /// <summary>Correction of one distance rule on a particle tied to a pinned one (share = 1).</summary>
        private static Vector3 RuleCorrection(Vector3 x, Vector3 anchor, float rest)
        {
            Vector3 d = x - anchor;
            float len = d.magnitude;
            return -(len - rest) * (d / len);
        }

        private void ResetChain()
        {
            var p = new SolverParams
            {
                substeps = 4, iterations = 4, relaxation = 1.5f, radius = 0.05f, damping = 0.15f,
                gaussSeidelDistance = false, multithreaded = false
            };
            chain = new UnifiedSolver(p);
            int phase = Phase.Make(chain.NewGroup());
            for (int k = 0; k < ChainCount; k++)
                chain.AddParticle(ChainAnchor + new Vector3(k * ChainSpacing, 0f, 0f), Vector3.zero, k == 0 ? 0f : 1f, phase);
            for (int k = 0; k + 1 < ChainCount; k++)
                chain.AddDistanceConstraint(k, k + 1);
            chainSteps = 0;
        }

        // ================================================================ timeline

        private void SetPart(int k)
        {
            part = k;
            time = 0f;
            if (k == 3)
                ResetChain();
        }

        private void Update()
        {
            var kb = Keyboard.current;
            if (kb != null && !recording)
            {
                if (kb.spaceKey.wasPressedThisFrame) paused = !paused;
                if (ClothShowcase.Pressed(kb, "r")) SetPart(0);
                if (kb.digit1Key.wasPressedThisFrame) SetPart(0);
                if (kb.digit2Key.wasPressedThisFrame) SetPart(1);
                if (kb.digit3Key.wasPressedThisFrame) SetPart(2);
                if (kb.digit4Key.wasPressedThisFrame) SetPart(3);
            }

            if (!paused)
                time += Time.deltaTime;
            if (time >= Durations[part])
            {
                if (part + 1 < Durations.Length) SetPart(part + 1);
                else if (recording) FinishRecording();
                else if (loop) SetPart(0);
                else time = Durations[part];
            }

            linesUsed = 0;
            textsUsed = 0;
            imagesUsed = 0;
            switch (part)
            {
                case 0: DrawDistance(time); break;
                case 1: DrawPinned(time); break;
                case 2: DrawTwoRules(time); break;
                case 3: DrawChain(time); break;
            }
            Label("GamiX engine  ·  running in Unity", new Vector2(0.97f, 0.04f), 24, Soft, TextAnchor.LowerRight);

            float d = Durations[part];
            float fade = Mathf.Max(1f - time / Fade, (time - (d - Fade)) / Fade);
            fadeOverlay.color = new Color(Background.r, Background.g, Background.b, Mathf.Clamp01(fade));
            fadeOverlay.transform.SetAsLastSibling();

            for (int i = linesUsed; i < lines.Count; i++) lines[i].enabled = false;
            for (int i = textsUsed; i < texts.Count; i++) texts[i].enabled = false;
            for (int i = imagesUsed; i < images.Count; i++) images[i].enabled = false;
        }

        // ================================================================ part 1

        private void DrawDistance(float t)
        {
            Title("1 · A distance constraint", "Two particles linked by a rule: stay at the rest length L.");
            Vector3 xi0 = distanceFrames[0][0], xj0 = distanceFrames[0][1];
            Vector3 di = distanceFrames[1][0] - xi0, dj = distanceFrames[1][1] - xj0;

            float grow = Ease(1.2f, 2.2f, t);
            float move = Ease(4.2f, 5.6f, t);
            float arrowAlpha = 1f - Ease(5.8f, 6.4f, t);
            Vector3 xi = xi0 + di * move, xj = xj0 + dj * move;

            Line(xi, xj, Link, 0.07f);
            RestBracket(0f, -0.55f, 2.6f);
            if (move > 0f)
            {
                Ring(xi0, BallSize * 0.5f, Soft, 0.025f);
                Ring(xj0, BallSize * 0.5f, Soft, 0.025f);
            }
            Ball(xi, "i", Teal);
            Ball(xj, "j", Teal);

            if (grow > 0f && arrowAlpha > 0f)
            {
                Color c = WithAlpha(Red, arrowAlpha);
                Arrow(xi0 + Vector3.up * 0.62f, xi0 + Vector3.up * 0.62f + di * grow, c);
                Arrow(xj0 + Vector3.up * 0.62f, xj0 + Vector3.up * 0.62f + dj * grow, c);
                WorldLabel("Δx<size=26>i</size>", xi0 + Vector3.up * 1.0f + di * 0.5f, 38, c);
                WorldLabel("Δx<size=26>j</size>", xj0 + Vector3.up * 1.0f + dj * 0.5f, 38, c);
            }

            float length = (xi - xj).magnitude, error = length - 2.6f;
            string state = error > 1e-3f ? "<color=#B02A2A>too long</color>" : "<color=#2E8B57>rule satisfied ✓</color>";
            Label($"C = |x<size=30>i</size> − x<size=30>j</size>| − L = {length:0.00} − 2.60 = <b>{Mathf.Max(0f, error):0.00}</b>   {state}",
                new Vector2(0.5f, 0.20f), 40, Ink, TextAnchor.MiddleCenter);
            if (t > 2.4f)
            {
                float a = Ease(2.4f, 2.9f, t);
                Label("Each particle moves along the link by its share of C:", new Vector2(0.59f, 0.115f), 32, WithAlpha(Soft, a), TextAnchor.MiddleRight);
                Formula(shareHalfFormula, new Vector2(0.60f, 0.115f), 0f, a);
            }
            if (t > 6.4f)
            {
                float a = Ease(6.4f, 6.9f, t);
                Label("Constraints move positions, not forces. The velocity comes afterwards:", new Vector2(0.66f, 0.70f), 32, WithAlpha(Ink, a), TextAnchor.MiddleRight);
                Formula(velocityFormula, new Vector2(0.67f, 0.70f), 0f, a);
            }
        }

        // ================================================================ part 2

        private void DrawPinned(float t)
        {
            Title("2 · A pinned particle: w = 0", "Inverse mass w = 1 / m. A pinned particle has infinite mass, so w = 0.");
            Vector3 xi0 = pinnedFrames[0][0], xj = pinnedFrames[0][1];
            Vector3 di = pinnedFrames[1][0] - xi0;

            float grow = Ease(1.2f, 2.2f, t);
            float move = Ease(4f, 5.4f, t);
            float arrowAlpha = 1f - Ease(5.6f, 6.2f, t);
            Vector3 xi = xi0 + di * move;

            Line(xi, xj, Link, 0.07f);
            RestBracket(xj.x - 1.1f, -0.55f, 2.2f);
            Anchor(xj);
            if (move > 0f)
                Ring(xi0, BallSize * 0.5f, Soft, 0.025f);
            Ball(xi, "i", Teal);
            Ball(xj, "j", Pinned);
            WorldLabel("w<size=26>j</size> = 0", xj + new Vector3(0.95f, 0f, 0f), 36, Pinned);

            if (grow > 0f && arrowAlpha > 0f)
            {
                Color c = WithAlpha(Red, arrowAlpha);
                Arrow(xi0 + Vector3.up * 0.62f, xi0 + Vector3.up * 0.62f + di * grow, c);
                WorldLabel("Δx<size=26>i</size>", xi0 + Vector3.up * 1.0f + di * 0.5f, 38, c);
                WorldLabel("Δx<size=26>j</size> = 0", xj + new Vector3(0f, -0.62f, 0f), 32, c);
            }

            float error = (xi - xj).magnitude - 2.2f;
            string state = error > 1e-3f ? "<color=#B02A2A>too long</color>" : "<color=#2E8B57>rule satisfied ✓</color>";
            Label($"C = <b>{Mathf.Max(0f, error):0.00}</b>   {state}", new Vector2(0.5f, 0.20f), 40, Ink, TextAnchor.MiddleCenter);
            if (t > 2.4f)
            {
                float a = Ease(2.4f, 2.9f, t);
                Label("Share of i:", new Vector2(0.36f, 0.115f), 32, WithAlpha(Soft, a), TextAnchor.MiddleRight);
                Formula(shareOneFormula, new Vector2(0.37f, 0.115f), 0f, a);
                Label("→   i takes the whole correction, j never moves.", new Vector2(0.475f, 0.115f), 32, WithAlpha(Soft, a), TextAnchor.MiddleLeft);
            }
        }

        // ================================================================ part 3

        private void DrawTwoRules(float t)
        {
            Title("3 · Two rules on the same particle", "The paper solves all rules in parallel (Jacobi), then combines them (figure 2).");
            const float start = 1.6f, period = 2f;
            int it = Mathf.Clamp(Mathf.FloorToInt((t - start) / period), -1, TwoRuleIterations - 1);
            float local = t - start - Mathf.Max(0, it) * period;
            float grow = it < 0 ? 0f : Ease(0f, 0.5f, local);
            float move = it < 0 ? 0f : Ease(0.8f, 1.4f, local);
            float arrowAlpha = it < 0 ? 0f : 1f - Ease(1.4f, 1.7f, local);
            int from = Mathf.Max(0, it);

            Label("<b>Summed</b>", new Vector2(0.06f, 0.775f), 34, Red, TextAnchor.MiddleLeft);
            Formula(summedFormula, new Vector2(0.06f, 0.705f), 0f, 1f);
            Label("<b>Our solver</b> (eq. 13)", new Vector2(0.06f, 0.435f), 34, Teal, TextAnchor.MiddleLeft);
            Formula(averagedFormula, new Vector2(0.06f, 0.36f), 0f, 1f);

            Vector3 summedFrom = summedPositions[from], summedTo = summedPositions[Mathf.Min(from + 1, TwoRuleIterations)];
            Vector3 avgFrom = averagedFrames[from][0], avgTo = averagedFrames[Mathf.Min(from + 1, TwoRuleIterations)][0];
            Vector3 summed = it < 0 ? summedFrom : Vector3.Lerp(summedFrom, summedTo, move);
            Vector3 averaged = it < 0 ? avgFrom : Vector3.Lerp(avgFrom, avgTo, move);

            DrawRuleRow(summed, summedFrom, new Vector3(AnchorX, RowTop, 0f), grow, arrowAlpha, move > 0f && it >= 0);
            DrawRuleRow(averaged, avgFrom, new Vector3(AnchorX, RowBottom, 0f), grow, arrowAlpha, move > 0f && it >= 0);

            if (it >= 0)
                Label($"iteration {it + 1}", new Vector2(0.94f, 0.86f), 34, Ink, TextAnchor.MiddleRight);
            if (it >= 1)
            {
                Label("never settles ✗", new Vector2(0.97f, 0.5f + RowTop / ViewHeight), 30, Red, TextAnchor.MiddleRight);
                Label("settles ✓", new Vector2(0.97f, 0.5f + RowBottom / ViewHeight), 30, Good, TextAnchor.MiddleRight);
            }
            if (t > start + 0.6f)
                Label("Each rule asks for the full correction. n = 2 rules on the particle → divide the sum by 2.",
                    new Vector2(0.5f, 0.12f), 32, WithAlpha(Soft, Ease(start + 0.6f, start + 1.1f, t)), TextAnchor.MiddleCenter);
        }

        private void DrawRuleRow(Vector3 x, Vector3 iterationStart, Vector3 anchor, float grow, float arrowAlpha, bool ghost)
        {
            Line(x + Vector3.up * 0.07f, anchor + Vector3.up * 0.07f, Link, 0.05f);
            Line(x - Vector3.up * 0.07f, anchor - Vector3.up * 0.07f, Link, 0.05f);
            RestBracket(anchor.x - TwoRuleRest * 0.5f, anchor.y - 0.45f, TwoRuleRest, false);
            Anchor(anchor);
            if (ghost)
                Ring(iterationStart, BallSize * 0.5f, Soft, 0.025f);
            Ball(x, "i", Teal);
            Ball(anchor, "j", Pinned);

            float error = (x - anchor).magnitude - TwoRuleRest;
            WorldLabel($"C = {error:+0.00;−0.00;0.00}", x + new Vector3(-0.9f, 0f, 0f), 28, Ink);

            if (grow > 0f && arrowAlpha > 0f)
            {
                Vector3 d = RuleCorrection(iterationStart, anchor, TwoRuleRest);
                Color c = WithAlpha(Red, arrowAlpha);
                Vector3 a = iterationStart + Vector3.up * 0.48f, b = iterationStart + Vector3.up * 0.66f;
                Arrow(a, a + d * grow, c, 0.045f);
                Arrow(b, b + d * grow, c, 0.045f);
            }
        }

        // ================================================================ part 4

        private void DrawChain(float t)
        {
            Title("4 · Our solver, every frame", "A hanging chain: gravity predicts x*, the rules pull the particles back.");
            const float release = 1.2f;
            int wanted = Mathf.Max(0, Mathf.FloorToInt((t - release) * 60f));
            while (chainSteps < wanted)
            {
                chain.Step(1f / 60f);
                chainSteps++;
            }

            Vector3[] x = chain.positions;
            Line(ChainAnchor, ChainAnchor + Vector3.up * 0.25f, Pinned, 0.035f);
            Line(ChainAnchor + new Vector3(-0.4f, 0.25f, 0f), ChainAnchor + new Vector3(0.4f, 0.25f, 0f), Pinned, 0.09f);
            float maxStretch = 0f;
            var rules = chain.DistanceConstraints;
            for (int k = 0; k < rules.Count; k++)
            {
                var r = rules.items[k];
                float stretch = (x[r.i] - x[r.j]).magnitude / r.restLength - 1f;
                maxStretch = Mathf.Max(maxStretch, stretch);
                Line(x[r.i], x[r.j], Color.Lerp(Link, Red, Mathf.Clamp01(stretch / 0.08f)), 0.06f);
            }
            for (int k = 0; k < ChainCount; k++)
                Ball(x[k], null, k == 0 ? Pinned : Teal, 0.2f);

            Label($"{ChainCount} particles · {rules.Count} distance rules · Jacobi + averaging · ω = {chain.parameters.relaxation:0.0} · " +
                  $"{chain.parameters.substeps} sub-steps × {chain.parameters.iterations} iterations",
                new Vector2(0.5f, 0.20f), 30, Ink, TextAnchor.MiddleCenter);
            Label($"longest link stretched by <b>{maxStretch * 100f:0.0} %</b>   ·   solve {chain.LastStepMilliseconds:0.00} ms",
                new Vector2(0.5f, 0.12f), 30, Soft, TextAnchor.MiddleCenter);
        }

        // ================================================================ drawing helpers

        private void Title(string title, string subtitle)
        {
            Label("<b>" + title + "</b>", new Vector2(0.05f, 0.92f), 52, Ink, TextAnchor.MiddleLeft);
            Label(subtitle, new Vector2(0.05f, 0.85f), 30, Soft, TextAnchor.MiddleLeft);
        }

        private void Ball(Vector3 p, string label, Color color, float size = BallSize)
        {
            Graphics.DrawMesh(sphere, RenderUtils.ScaleTranslation(p, size), RenderUtils.GetSharedMaterial(color), 0);
            if (!string.IsNullOrEmpty(label))
                WorldLabel("<b>" + label + "</b>", p, 40, Color.white);
        }

        /// <summary>Ceiling bar above a pinned particle.</summary>
        private void Anchor(Vector3 p)
        {
            Vector3 top = p + Vector3.up * 0.62f;
            Line(p, top, Pinned, 0.035f);
            Line(top + Vector3.left * 0.34f, top + Vector3.right * 0.34f, Pinned, 0.09f);
            for (int k = 0; k < 5; k++)
            {
                Vector3 a = top + new Vector3(-0.3f + k * 0.15f, 0.05f, 0f);
                Line(a, a + new Vector3(0.1f, 0.13f, 0f), Pinned, 0.025f);
            }
        }

        /// <summary>Dashed bracket of the rest length L.</summary>
        private void RestBracket(float centerX, float y, float length, bool label = true)
        {
            float a = centerX - length * 0.5f, b = centerX + length * 0.5f;
            const int dashes = 14;
            for (int k = 0; k < dashes; k++)
            {
                float u0 = (float)k / dashes, u1 = (k + 0.55f) / dashes;
                Line(new Vector3(Mathf.Lerp(a, b, u0), y, 0f), new Vector3(Mathf.Lerp(a, b, u1), y, 0f), Soft, 0.03f);
            }
            Line(new Vector3(a, y - 0.09f, 0f), new Vector3(a, y + 0.09f, 0f), Soft, 0.03f);
            Line(new Vector3(b, y - 0.09f, 0f), new Vector3(b, y + 0.09f, 0f), Soft, 0.03f);
            if (label)
                WorldLabel("rest length L", new Vector3(centerX, y - 0.24f, 0f), 26, Soft);
        }

        private void Arrow(Vector3 from, Vector3 to, Color color, float width = 0.055f)
        {
            Vector3 d = to - from;
            float len = d.magnitude;
            if (len < 1e-3f) return;
            Vector3 dir = d / len;
            float head = Mathf.Min(0.24f, len * 0.6f);
            Vector3 z = new Vector3(0f, 0f, -0.7f);
            Line(from + z, to - dir * head * 0.9f + z, color, width);
            LineRenderer lr = NextLine();
            lr.positionCount = 2;
            lr.SetPosition(0, to - dir * head + z);
            lr.SetPosition(1, to + z);
            lr.widthCurve = AnimationCurve.Linear(0f, 1f, 1f, 0f);
            lr.widthMultiplier = width * 4f;
            lr.numCapVertices = 0;
            lr.startColor = lr.endColor = color;
        }

        private void Line(Vector3 a, Vector3 b, Color color, float width)
        {
            LineRenderer lr = NextLine();
            lr.positionCount = 2;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
            lr.widthCurve = AnimationCurve.Constant(0f, 1f, 1f);
            lr.widthMultiplier = width;
            lr.numCapVertices = 4;
            lr.startColor = lr.endColor = color;
        }

        private void Ring(Vector3 center, float radius, Color color, float width)
        {
            const int segments = 24;
            for (int k = 0; k < segments; k += 2)
            {
                float a0 = k * Mathf.PI * 2f / segments, a1 = (k + 1) * Mathf.PI * 2f / segments;
                Line(center + new Vector3(Mathf.Cos(a0), Mathf.Sin(a0), 0f) * radius,
                    center + new Vector3(Mathf.Cos(a1), Mathf.Sin(a1), 0f) * radius, color, width);
            }
        }

        private LineRenderer NextLine()
        {
            if (linesUsed == lines.Count)
            {
                var go = new GameObject("Line");
                go.transform.SetParent(transform, false);
                var lr = go.AddComponent<LineRenderer>();
                lr.sharedMaterial = lineMaterial;
                lr.useWorldSpace = true;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                lines.Add(lr);
            }
            LineRenderer l = lines[linesUsed++];
            l.enabled = true;
            return l;
        }

        private void WorldLabel(string text, Vector3 world, int size, Color color)
        {
            Vector3 v = cam.WorldToViewportPoint(world);
            Label(text, new Vector2(v.x, v.y), size, color, TextAnchor.MiddleCenter);
        }

        private void Label(string text, Vector2 viewport, int size, Color color, TextAnchor anchor)
        {
            if (textsUsed == texts.Count)
            {
                var go = new GameObject("Text", typeof(RectTransform));
                go.transform.SetParent(canvas.transform, false);
                var tx = go.AddComponent<Text>();
                tx.font = font;
                tx.supportRichText = true;
                tx.horizontalOverflow = HorizontalWrapMode.Overflow;
                tx.verticalOverflow = VerticalWrapMode.Overflow;
                tx.raycastTarget = false;
                tx.rectTransform.sizeDelta = new Vector2(10f, 10f);
                texts.Add(tx);
            }
            Text t = texts[textsUsed++];
            t.enabled = true;
            t.text = text;
            t.fontSize = size;
            t.color = color;
            t.alignment = anchor;
            RectTransform rt = t.rectTransform;
            rt.anchorMin = rt.anchorMax = viewport;
            rt.pivot = Pivot(anchor);
            rt.anchoredPosition = Vector2.zero;
        }

        /// <summary>Rendered formula image, left edge at the viewport point. Height in reference pixels (1080 high); 0 = the image's own
        /// pixel size (the PNGs are rendered at the size they are shown, so they stay sharp).</summary>
        private void Formula(Texture2D tex, Vector2 viewport, float height, float alpha)
        {
            if (tex == null) return;
            if (imagesUsed == images.Count)
            {
                var go = new GameObject("Formula", typeof(RectTransform));
                go.transform.SetParent(canvas.transform, false);
                var ri = go.AddComponent<RawImage>();
                ri.raycastTarget = false;
                images.Add(ri);
            }
            RawImage img = images[imagesUsed++];
            img.enabled = true;
            img.texture = tex;
            img.color = new Color(1f, 1f, 1f, alpha);
            RectTransform rt = img.rectTransform;
            rt.anchorMin = rt.anchorMax = viewport;
            rt.pivot = new Vector2(0f, 0.5f);
            rt.anchoredPosition = Vector2.zero;
            if (height <= 0f) height = tex.height;
            rt.sizeDelta = new Vector2(height * tex.width / tex.height, height);
        }

        private static Vector2 Pivot(TextAnchor a)
        {
            switch (a)
            {
                case TextAnchor.MiddleLeft: return new Vector2(0f, 0.5f);
                case TextAnchor.MiddleRight: return new Vector2(1f, 0.5f);
                case TextAnchor.LowerRight: return new Vector2(1f, 0f);
                case TextAnchor.UpperLeft: return new Vector2(0f, 1f);
                default: return new Vector2(0.5f, 0.5f);
            }
        }

        private void BuildCanvas()
        {
            font = Font.CreateDynamicFontFromOSFont(new[] { "Segoe UI", "Arial" }, 48);
            var go = new GameObject("Explainer Canvas");
            go.transform.SetParent(transform, false);
            canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = cam;
            canvas.planeDistance = 1f;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 1f;

            var overlay = new GameObject("Fade", typeof(RectTransform));
            overlay.transform.SetParent(go.transform, false);
            fadeOverlay = overlay.AddComponent<Image>();
            fadeOverlay.raycastTarget = false;
            RectTransform rt = fadeOverlay.rectTransform;
            rt.anchorMin = Vector2.zero;
            rt.anchorMax = Vector2.one;
            rt.offsetMin = rt.offsetMax = Vector2.zero;
        }

        private void BuildSphere()
        {
            ParticleFactory.Icosphere(3, out List<Vector3> verts, out List<int> tris);
            var v = new Vector3[verts.Count];
            var n = new Vector3[verts.Count];
            for (int i = 0; i < v.Length; i++)
            {
                n[i] = verts[i].normalized;
                v[i] = n[i] * 0.5f;
            }
            sphere = new Mesh { name = "Explainer Sphere", vertices = v, normals = n, triangles = tris.ToArray() };
            sphere.bounds = new Bounds(Vector3.zero, Vector3.one);
        }

        private static float Ease(float a, float b, float t)
        {
            float u = Mathf.Clamp01((t - a) / (b - a));
            return u * u * (3f - 2f * u);
        }

        private static Color WithAlpha(Color c, float a) => new Color(c.r, c.g, c.b, c.a * a);

        private static Color Hex(int rgb) =>
            new Color(((rgb >> 16) & 255) / 255f, ((rgb >> 8) & 255) / 255f, (rgb & 255) / 255f);

        // ================================================================ recording

        private void StartRecording()
        {
            Directory.CreateDirectory(recordFolder);
            foreach (string f in Directory.GetFiles(recordFolder, "*.jpg"))
                File.Delete(f);
            string done = Path.Combine(recordFolder, "done.txt");
            if (File.Exists(done)) File.Delete(done);
            target = new RenderTexture(recordWidth, recordHeight, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                antiAliasing = 8
            };
            readback = new Texture2D(recordWidth, recordHeight, TextureFormat.RGB24, false);
            cam.targetTexture = target;
            Time.captureFramerate = recordFps;
            Application.runInBackground = true;   // keep the editor's play mode running when its window is not focused
            recording = true;
            frame = 0;
            StartCoroutine(CaptureFrames());
        }

        private IEnumerator CaptureFrames()
        {
            var endOfFrame = new WaitForEndOfFrame();
            while (recording)
            {
                yield return endOfFrame;
                if (!recording) break;
                RenderTexture.active = target;
                readback.ReadPixels(new Rect(0, 0, recordWidth, recordHeight), 0, 0, false);
                RenderTexture.active = null;
                File.WriteAllBytes(Path.Combine(recordFolder, $"frame_{frame:0000}.jpg"), readback.EncodeToJPG(94));
                frame++;
            }
        }

        private void FinishRecording()
        {
            recording = false;
            Time.captureFramerate = 0;
            cam.targetTexture = null;
            File.WriteAllText(Path.Combine(recordFolder, "done.txt"), frame.ToString());
            time = Durations[part];
        }
    }
}
