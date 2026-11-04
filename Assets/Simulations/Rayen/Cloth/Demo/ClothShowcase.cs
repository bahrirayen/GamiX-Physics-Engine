using System.Collections.Generic;
using GamiX.Physics.Components;
using GamiX.Physics.Demos;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Cloth showcase (paper section 8): a textured flag in gusty wind (aerodynamics, tethers), a tartan curtain that
    /// tears when it is pulled or cut (real mesh splitting), and a sheet that falls on obstacles.
    ///
    /// Controls: left drag = grab and pull, hold C = cut, T = rip the curtain apart, J = Jacobi / Gauss-Seidel
    /// distance constraints, W = wind, V = particle view, 1-4 = camera shots, R = reset, H = hide the text,
    /// Space = impulse, P = pause. Letters are read by their label, so AZERTY and QWERTY keyboards both work.
    /// </summary>
    public sealed class ClothShowcase : MonoBehaviour
    {
        public Vector3 wind = new Vector3(6f, 0f, 2.5f);
        [Tooltip("Relative strength of the gusts (0 = steady wind).")]
        [Range(0f, 1f)] public float gustiness = 0.45f;
        public bool showText = true;

        private UnifiedWorld world;
        private ClothAerodynamics aero;
        private ParticleCloth flag, curtain, sheet;
        private ClothCutter cutter;
        private OrbitCamera orbit;
        private bool windOn = true;
        private bool particleView;
        private float smoothMs, smoothFps = 60f, gustPercent, stretchNow;
        private int tornEdges, splitVertices;

        // Stretch history for the graph: one sample every 0.1 s, the last 6 s.
        private const int HistoryLength = 60;
        private readonly float[] stretchHistory = new float[HistoryLength];
        private readonly bool[] jacobiHistory = new bool[HistoryLength];
        private int historyHead;
        private float historyTimer;
        private readonly List<Matrix4x4> particleBatch = new List<Matrix4x4>(1023);
        private readonly Matrix4x4[] particleBuffer = new Matrix4x4[1023];

        // Rip: an invisible blade runs up the middle of the curtain, then the two halves are pushed apart.
        private float ripTime = -1f;
        private int ripRow;
        private const float RipDuration = 1.2f;

        private struct Shot
        {
            public Vector3 target;
            public float distance, yaw, pitch;
        }

        private static readonly Shot[] Shots =
        {
            new Shot { target = new Vector3(0.2f, 1.3f, 0f), distance = 8.5f, yaw = 0f, pitch = 14f },     // 1 wide
            new Shot { target = new Vector3(-2.6f, 2.3f, 0.2f), distance = 3.6f, yaw = 25f, pitch = 8f },  // 2 flag
            new Shot { target = new Vector3(0.4f, 1.4f, 0f), distance = 3.8f, yaw = -8f, pitch = 6f },     // 3 curtain
            new Shot { target = new Vector3(3.3f, 0.7f, 0f), distance = 3.6f, yaw = -30f, pitch = 28f },   // 4 sheet
        };
        private int shot;

        private void Awake()
        {
            world = DemoUtils.CreateUnifiedWorld();
            world.parameters.radius = 0.03f;
            world.parameters.substeps = 3;
            world.parameters.iterations = 4;
            world.parameters.wind = wind;
            world.showStats = false;
            aero = world.gameObject.AddComponent<ClothAerodynamics>();
            aero.drag = 1.5f;
            aero.lift = 0.8f;

            BuildStage();

            // Flag on a pole (no tearing: the tethers keep it from stretching under the wind).
            DemoUtils.Visual(PrimitiveType.Cylinder, "Pole", new Vector3(-3.4f, 1.6f, 0f), new Vector3(0.06f, 1.6f, 0.06f), new Color(0.55f, 0.56f, 0.6f));
            DemoUtils.Visual(PrimitiveType.Sphere, "Pole Cap", new Vector3(-3.4f, 3.22f, 0f), Vector3.one * 0.12f, new Color(0.78f, 1f, 0.1f));
            flag = DemoUtils.Body<ParticleCloth>(world, "Flag", new Vector3(-2.65f, 2.55f, 0f), Color.white);
            flag.transform.rotation = Quaternion.Euler(-90f, 0f, 0f); // cloth plane XZ -> vertical XY, texture upright
            flag.dimensions = new Vector2(1.5f, 1f);
            flag.resolution = new Vector2Int(26, 18);
            flag.pinMode = ClothPinMode.LeftEdge;
            flag.particleMass = 0.005f;
            flag.texture = ClothTextures.Flag();
            flag.cuttable = false;

            // Curtain hanging from a beam: tears when pulled hard, or with the blade.
            DemoUtils.Visual(PrimitiveType.Cube, "Beam", new Vector3(0.4f, 2.62f, 0f), new Vector3(2.4f, 0.08f, 0.08f), new Color(0.35f, 0.25f, 0.18f));
            DemoUtils.Visual(PrimitiveType.Cube, "Post L", new Vector3(-0.8f, 1.31f, 0f), new Vector3(0.08f, 2.62f, 0.08f), new Color(0.35f, 0.25f, 0.18f));
            DemoUtils.Visual(PrimitiveType.Cube, "Post R", new Vector3(1.6f, 1.31f, 0f), new Vector3(0.08f, 2.62f, 0.08f), new Color(0.35f, 0.25f, 0.18f));
            curtain = DemoUtils.Body<ParticleCloth>(world, "Curtain", new Vector3(0.4f, 1.62f, 0f), Color.white);
            curtain.transform.rotation = Quaternion.Euler(-90f, 0f, 0f); // local +Z -> up, so the top edge is v = last
            curtain.dimensions = new Vector2(2.1f, 1.9f);
            curtain.resolution = new Vector2Int(32, 29);
            curtain.pinMode = ClothPinMode.TopEdge;
            curtain.particleMass = 0.035f; // heavy fabric: hangs and sways instead of flying
            curtain.useTethers = false; // a torn piece must be free to fall
            curtain.tearRatio = 1.45f;
            curtain.texture = ClothTextures.Tartan();

            // Sheet falling on a sphere and a box.
            DemoUtils.Collider(world, "Sphere Collider", ColliderShape.Sphere, new Vector3(3.1f, 0.5f, 0f), new Vector3(0.45f, 0.45f, 0.45f));
            DemoUtils.Visual(PrimitiveType.Sphere, "Sphere", new Vector3(3.1f, 0.5f, 0f), Vector3.one * 0.88f, new Color(0.2f, 0.55f, 0.45f));
            DemoUtils.Collider(world, "Box Collider", ColliderShape.Box, new Vector3(3.75f, 0.3f, 0.35f), new Vector3(0.6f, 0.6f, 0.6f));
            DemoUtils.Visual(PrimitiveType.Cube, "Box", new Vector3(3.75f, 0.3f, 0.35f), Vector3.one * 0.58f, new Color(0.85f, 0.55f, 0.2f));
            sheet = DemoUtils.Body<ParticleCloth>(world, "Sheet", new Vector3(3.35f, 1.7f, 0.15f), Color.white);
            sheet.transform.rotation = Quaternion.Euler(0f, 20f, 0f);
            sheet.dimensions = new Vector2(1.6f, 1.6f);
            sheet.resolution = new Vector2Int(28, 28);
            sheet.pinMode = ClothPinMode.None;
            sheet.useTethers = false;
            sheet.aerodynamics = false;
            sheet.tearRatio = 1.6f;
            sheet.texture = ClothTextures.Gingham();

            world.gameObject.SetActive(true);
            flag.gameObject.SetActive(true);
            curtain.gameObject.SetActive(true);
            sheet.gameObject.SetActive(true);
        }

        private void Start()
        {
            Camera cam = Camera.main;
            if (cam != null)
            {
                orbit = cam.GetComponent<OrbitCamera>();
                cutter = cam.GetComponent<ClothCutter>();
                if (cutter == null) cutter = cam.gameObject.AddComponent<ClothCutter>();
                cutter.world = world;
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = new Color(0.16f, 0.18f, 0.22f);
            }
            world.BeforeStep += OnBeforeStep;
            ApplyShot(0);
        }

        private void BuildStage()
        {
            DemoUtils.Collider(world, "Ground Collider", ColliderShape.Plane, Vector3.zero, Vector3.one);
            GameObject ground = DemoUtils.Visual(PrimitiveType.Plane, "Ground", Vector3.zero, new Vector3(3f, 1f, 3f), Color.white);
            Material m = ground.GetComponent<MeshRenderer>().sharedMaterial;
            Texture2D floor = ClothTextures.Floor();
            if (m.HasProperty("_BaseMap")) { m.SetTexture("_BaseMap", floor); m.SetTextureScale("_BaseMap", new Vector2(12f, 12f)); }
            if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.25f);
        }

        private void Update()
        {
            Keyboard kb = Keyboard.current;
            if (kb != null)
            {
                if (Pressed(kb, "j")) world.parameters.gaussSeidelDistance = !world.parameters.gaussSeidelDistance;
                if (Pressed(kb, "w")) windOn = !windOn;
                if (Pressed(kb, "t")) Rip();
                if (Pressed(kb, "h")) showText = !showText;
                if (Pressed(kb, "r")) Reload();
                if (Pressed(kb, "v")) SetParticleView(!particleView);
                if (kb.digit1Key.wasPressedThisFrame) ApplyShot(0);
                if (kb.digit2Key.wasPressedThisFrame) ApplyShot(1);
                if (kb.digit3Key.wasPressedThisFrame) ApplyShot(2);
                if (kb.digit4Key.wasPressedThisFrame) ApplyShot(3);
            }

            // Gusty wind: slow changes of strength and direction from Perlin noise.
            float t = Time.time;
            float gust = 1f + gustiness * (2f * Mathf.PerlinNoise(t * 0.35f, 0.3f) - 1f);
            gustPercent = (gust - 1f) * 100f;
            float swirl = gustiness * (Mathf.PerlinNoise(0.7f, t * 0.25f) - 0.5f);
            Vector3 w = Quaternion.Euler(0f, swirl * 40f, 0f) * wind * gust;
            world.parameters.wind = windOn ? w : Vector3.zero;

            if (world.Solver != null)
            {
                smoothMs = Mathf.Lerp(smoothMs, (float)world.Solver.LastStepMilliseconds, 0.1f);
                smoothFps = Mathf.Lerp(smoothFps, 1f / Mathf.Max(1e-4f, Time.unscaledDeltaTime), 0.05f);
                SampleLiveValues();
            }
        }

        /// <summary>
        /// Key by the letter printed on it, in the current keyboard layout (the Input System's Key enum is the
        /// physical position on a US keyboard: on AZERTY, Key.W is the key labeled Z).
        /// </summary>
        public static bool Pressed(Keyboard kb, string label)
        {
            var key = kb.FindKeyOnCurrentKeyboardLayout(label);
            return key != null && key.wasPressedThisFrame;
        }

        /// <summary>
        /// Particle view (like the paper's fig. 17): hides the cloth meshes and draws the particles they are made of.
        /// </summary>
        public void SetParticleView(bool on)
        {
            particleView = on;
            foreach (ParticleCloth c in new[] { flag, curtain, sheet })
                if (c != null) c.GetComponent<MeshRenderer>().enabled = !on;
        }

        private void LateUpdate()
        {
            if (!particleView || world == null || world.Solver == null) return;
            UnifiedSolver s = world.Solver;
            float d = 2f * s.parameters.radius;
            DrawParticles(s, flag, new Color(0.1f, 0.75f, 0.8f), d);
            DrawParticles(s, curtain, new Color(0.8f, 0.2f, 0.2f), d);
            DrawParticles(s, sheet, new Color(0.6f, 0.75f, 0.95f), d);
        }

        private void DrawParticles(UnifiedSolver s, ParticleCloth cloth, Color color, float diameter)
        {
            if (cloth == null || cloth.Tearing == null) return;
            var rp = new RenderParams(RenderUtils.GetSharedMaterial(color));
            Mesh sphere = RenderUtils.SphereMesh;
            particleBatch.Clear();
            for (int k = 0; k < cloth.Tearing.MeshVertexCount; k++)
            {
                particleBatch.Add(RenderUtils.ScaleTranslation(s.positions[cloth.Tearing.MeshParticle(k)], diameter));
                if (particleBatch.Count == particleBuffer.Length)
                {
                    particleBatch.CopyTo(particleBuffer);
                    Graphics.RenderMeshInstanced(rp, sphere, 0, particleBuffer, particleBatch.Count);
                    particleBatch.Clear();
                }
            }
            if (particleBatch.Count == 0) return;
            particleBatch.CopyTo(particleBuffer);
            Graphics.RenderMeshInstanced(rp, sphere, 0, particleBuffer, particleBatch.Count);
        }

        /// <summary>Rips the curtain from the bottom up along a slightly wavy line (repeatable, for recording).</summary>
        public void Rip()
        {
            if (curtain == null || curtain.Tearing == null || ripTime >= 0f) return;
            ripTime = 0f;
            ripRow = 0;
        }

        private int RipColumn(int row) => curtain.resolution.x / 2 + Mathf.RoundToInt(1.5f * Mathf.Sin(row * 0.55f));

        private void OnBeforeStep(UnifiedSolver s, float h)
        {
            if (ripTime < 0f) return;
            ripTime += h;
            int resU = curtain.resolution.x, lastRow = curtain.resolution.y - 4; // stop below the beam: both halves keep hanging
            int target = Mathf.Min(lastRow, Mathf.FloorToInt(lastRow * ripTime / RipDuration));
            for (; ripRow < target; ripRow++)
            {
                Vector3 a = s.positions[curtain.Cloth.particles[ripRow * resU + RipColumn(ripRow)]];
                Vector3 b = s.positions[curtain.Cloth.particles[(ripRow + 1) * resU + RipColumn(ripRow + 1)]];
                curtain.Cut(a, b);
            }
            if (ripRow < lastRow) return;
            // Push the two halves apart like stage curtains.
            float center = curtain.transform.position.x;
            for (int k = 0; k < curtain.Tearing.MeshVertexCount; k++)
            {
                int i = curtain.Tearing.MeshParticle(k);
                if (s.invMass[i] == 0f) continue;
                float side = s.restPositions[i].x < center ? -1f : 1f;
                s.velocities[i] += new Vector3(1.2f * side, 0.3f, -0.6f);
            }
            ripTime = -1f;
        }

        private void OnDestroy()
        {
            if (world != null) world.BeforeStep -= OnBeforeStep;
        }

        private static void Reload()
        {
            Scene scene = SceneManager.GetActiveScene();
#if UNITY_EDITOR
            // Works in the editor even when the scene is not in the build settings.
            UnityEditor.SceneManagement.EditorSceneManager.LoadSceneInPlayMode(scene.path, new LoadSceneParameters(LoadSceneMode.Single));
#else
            SceneManager.LoadScene(scene.buildIndex);
#endif
        }

        private void ApplyShot(int i)
        {
            shot = i;
            if (orbit == null) return;
            orbit.target = Shots[i].target;
            orbit.distance = Shots[i].distance;
            orbit.yaw = Shots[i].yaw;
            orbit.pitch = Shots[i].pitch;
        }

        private void SampleLiveValues()
        {
            UnifiedSolver solver = world.Solver;
            tornEdges = splitVertices = 0;
            foreach (ParticleCloth c in new[] { flag, curtain, sheet })
            {
                if (c == null || c.Tearing == null) continue;
                tornEdges += c.Tearing.TornEdgeCount;
                splitVertices += c.Tearing.SplitCount;
            }
            stretchNow = flag != null && flag.Cloth != null ? flag.MaxStretch(solver) * 100f : 0f;
            historyTimer += Time.deltaTime;
            if (historyTimer < 0.1f) return;
            historyTimer = 0f;
            stretchHistory[historyHead] = stretchNow;
            jacobiHistory[historyHead] = !world.parameters.gaussSeidelDistance;
            historyHead = (historyHead + 1) % HistoryLength;
        }

        // ================================================================ on-screen panels

        private const float RefHeight = 1080f;   // the panels are laid out for 1080p and scaled to the screen
        private static readonly Color Panel = new Color(0.04f, 0.05f, 0.07f, 0.62f);
        private static readonly Color Accent = new Color(0.78f, 1f, 0.1f);
        private static readonly Color Warm = new Color(1f, 0.55f, 0.2f);
        private GUIStyle titleStyle, headerStyle, keyStyle, textStyle, valueStyle, smallStyle;

        private void MakeStyles()
        {
            if (titleStyle != null) return;
            textStyle = new GUIStyle(GUI.skin.label) { fontSize = 19, richText = true };
            textStyle.normal.textColor = new Color(0.86f, 0.89f, 0.93f);
            titleStyle = new GUIStyle(textStyle) { fontSize = 28, fontStyle = FontStyle.Bold };
            titleStyle.normal.textColor = Color.white;
            headerStyle = new GUIStyle(textStyle) { fontSize = 15, fontStyle = FontStyle.Bold };
            headerStyle.normal.textColor = Accent;
            keyStyle = new GUIStyle(textStyle) { fontStyle = FontStyle.Bold };
            keyStyle.normal.textColor = Color.white;
            valueStyle = new GUIStyle(textStyle) { fontStyle = FontStyle.Bold, alignment = TextAnchor.UpperLeft };
            valueStyle.normal.textColor = Color.white;
            smallStyle = new GUIStyle(textStyle) { fontSize = 14 };
            smallStyle.normal.textColor = new Color(0.65f, 0.7f, 0.76f);
        }

        private void OnGUI()
        {
            if (!showText || world == null || world.Solver == null) return;
            MakeStyles();
            float scale = Screen.height / RefHeight;
            Matrix4x4 baseMatrix = Matrix4x4.Scale(new Vector3(scale, scale, 1f));
            GUI.matrix = baseMatrix;
            float width = Screen.width / scale;
            DrawControls();
            DrawLive(width, baseMatrix);
            GUI.matrix = Matrix4x4.identity;
        }

        private void DrawControls()
        {
            var r = new Rect(24, 24, 370, 570);
            Box(r);
            float x = r.x + 22, y = r.y + 16;
            GUI.Label(new Rect(x, y, 300, 36), "GamiX Cloth", titleStyle);
            y += 36;
            GUI.Label(new Rect(x, y, 300, 22), "Unified particle physics  ·  paper §8", smallStyle);
            y += 40;

            y = Section(x, y, "MOUSE", new[]
            {
                ("Left drag", "pull the cloth"),
                ("Right drag", "orbit"),
                ("Middle drag", "pan"),
                ("Wheel", "zoom"),
            });
            y += 14;
            Section(x, y, "KEYS", new[]
            {
                ("C  (hold)", "cut with the blade"),
                ("T", "rip the curtain"),
                ("J", "Jacobi / Gauss-Seidel"),
                ("W", "wind on / off"),
                ("V", "particles / mesh"),
                ("1 - 4", "camera shots"),
                ("Space", "impulse"),
                ("P", "pause"),
                ("R", "reset"),
                ("H", "hide panels"),
            });
        }

        private float Section(float x, float y, string header, (string key, string action)[] rows)
        {
            GUI.Label(new Rect(x, y, 300, 22), header, headerStyle);
            y += 26;
            foreach (var (key, action) in rows)
            {
                GUI.Label(new Rect(x, y, 120, 26), key, keyStyle);
                GUI.Label(new Rect(x + 122, y, 220, 26), action, textStyle);
                y += 28;
            }
            return y;
        }

        private void DrawLive(float screenWidth, Matrix4x4 baseMatrix)
        {
            var r = new Rect(screenWidth - 24 - 390, 24, 390, 520);
            Box(r);
            var p = world.parameters;
            float x = r.x + 22, y = r.y + 16, vx = x + 128;
            GUI.Label(new Rect(x, y, 340, 22), "LIVE", headerStyle);
            y += 32;

            // Solver
            GUI.Label(new Rect(x, y, 120, 26), "Solver", textStyle);
            GUI.Label(new Rect(vx, y, 230, 26), p.gaussSeidelDistance ? "Gauss-Seidel" : $"<color=#ff8c33>Jacobi</color>  (paper)", valueStyle);
            y += 26;
            GUI.Label(new Rect(vx, y, 230, 22), $"{p.substeps} sub-steps × {p.iterations} iterations", smallStyle);
            y += 34;

            // Wind with an arrow seen from the camera
            GUI.Label(new Rect(x, y, 120, 26), "Wind", textStyle);
            if (windOn)
            {
                GUI.Label(new Rect(vx, y, 120, 26), $"{p.wind.magnitude:F1} m/s", valueStyle);
                GUI.Label(new Rect(vx, y + 26, 160, 22), $"gust {gustPercent:+0;-0} %", smallStyle);
                DrawWindArrow(new Vector2(r.xMax - 62, y + 22), 26f, baseMatrix);
            }
            else
            {
                GUI.Label(new Rect(vx, y, 120, 26), "<color=#8a929c>off</color>", valueStyle);
            }
            y += 62;

            // Stretch and its history
            GUI.Label(new Rect(x, y, 120, 26), "Stretch", textStyle);
            GUI.Label(new Rect(vx, y, 120, 26), $"{stretchNow:F1} %", valueStyle);
            GUI.Label(new Rect(vx + 92, y + 3, 150, 22), "flag, max edge", smallStyle);
            y += 32;
            DrawStretchGraph(new Rect(x, y, r.width - 44, 70));
            y += 70 + 22;

            // Tearing and cost
            Row(x, vx, ref y, "Torn", $"{tornEdges} edges");
            Row(x, vx, ref y, "Split", $"{splitVertices} vertices");
            Row(x, vx, ref y, "Particles", $"{world.Solver.ActiveCount:N0}");
            Row(x, vx, ref y, "Physics", $"{smoothMs:F1} ms   ·   {smoothFps:F0} FPS");
            if (cutter != null && cutter.Cutting)
                GUI.Label(new Rect(x, y + 4, 300, 26), "<color=#c8ff00>CUTTING</color>", valueStyle);
            if (world.paused)
                GUI.Label(new Rect(x + 140, y + 4, 200, 26), "<color=#ff8c33>PAUSED</color>", valueStyle);
        }

        private void Row(float x, float vx, ref float y, string label, string value)
        {
            GUI.Label(new Rect(x, y, 120, 26), label, textStyle);
            GUI.Label(new Rect(vx, y, 240, 26), value, valueStyle);
            y += 32;
        }

        private void DrawStretchGraph(Rect g)
        {
            const float maxPercent = 25f;
            Fill(g, new Color(1f, 1f, 1f, 0.05f));
            // Reference line at 10 %.
            float y10 = g.yMax - g.height * (10f / maxPercent);
            Fill(new Rect(g.x, y10, g.width, 1f), new Color(1f, 1f, 1f, 0.25f));
            GUI.Label(new Rect(g.xMax - 40, y10 - 18, 40, 18), "10 %", smallStyle);
            float bar = g.width / HistoryLength;
            for (int k = 0; k < HistoryLength; k++)
            {
                int i = (historyHead + k) % HistoryLength; // oldest first
                float h = g.height * Mathf.Clamp01(stretchHistory[i] / maxPercent);
                if (h <= 0f) continue;
                Fill(new Rect(g.x + k * bar, g.yMax - h, Mathf.Max(1f, bar - 1f), h), jacobiHistory[i] ? Warm : Accent);
            }
            GUI.Label(new Rect(g.x, g.yMax + 2, g.width, 20),
                      "<color=#c8ff00>■</color> Gauss-Seidel    <color=#ff8c33>■</color> Jacobi    last 6 s", smallStyle);
        }

        /// <summary>Wind direction as seen from the camera (right = to the right of the screen, up = away).</summary>
        private void DrawWindArrow(Vector2 center, float radius, Matrix4x4 baseMatrix)
        {
            Camera cam = Camera.main;
            Vector3 w = world.parameters.wind;
            if (cam == null || w.sqrMagnitude < 1e-6f) return;
            Vector3 right = cam.transform.right, forward = Vector3.ProjectOnPlane(cam.transform.forward, Vector3.up).normalized;
            var dir = new Vector2(Vector3.Dot(w, right), -Vector3.Dot(w, forward)).normalized; // GUI y points down
            Fill(new Rect(center.x - radius - 6, center.y - radius - 6, 2 * radius + 12, 2 * radius + 12), new Color(1f, 1f, 1f, 0.05f));
            Vector2 tail = center - dir * radius, tip = center + dir * radius;
            Line(tail, tip, 4f, Accent, baseMatrix);
            Vector2 back = -dir * 12f, side = new Vector2(-dir.y, dir.x) * 8f;
            Line(tip, tip + back + side, 4f, Accent, baseMatrix);
            Line(tip, tip + back - side, 4f, Accent, baseMatrix);
        }

        private static void Line(Vector2 a, Vector2 b, float width, Color color, Matrix4x4 baseMatrix)
        {
            Vector2 d = b - a;
            float angle = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;
            GUI.matrix = baseMatrix;
            // RotateAroundPivot takes screen coordinates: convert the pivot through the scale.
            GUIUtility.RotateAroundPivot(angle, baseMatrix.MultiplyPoint3x4(a));
            Fill(new Rect(a.x, a.y - width * 0.5f, d.magnitude, width), color);
            GUI.matrix = baseMatrix;
        }

        private static void Box(Rect r) => Fill(r, Panel);

        private static void Fill(Rect r, Color c)
        {
            Color old = GUI.color;
            GUI.color = c;
            GUI.DrawTexture(r, Texture2D.whiteTexture);
            GUI.color = old;
        }
    }
}
