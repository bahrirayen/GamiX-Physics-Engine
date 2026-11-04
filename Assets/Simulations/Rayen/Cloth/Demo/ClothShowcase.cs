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
    /// distance constraints, W = wind, 1-4 = camera shots, R = reset, H = hide the text, Space = impulse, P = pause.
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
        private float smoothMs;

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
                if (kb.jKey.wasPressedThisFrame) world.parameters.gaussSeidelDistance = !world.parameters.gaussSeidelDistance;
                if (kb.wKey.wasPressedThisFrame) windOn = !windOn;
                if (kb.tKey.wasPressedThisFrame) Rip();
                if (kb.hKey.wasPressedThisFrame) showText = !showText;
                if (kb.rKey.wasPressedThisFrame) Reload();
                if (kb.digit1Key.wasPressedThisFrame) ApplyShot(0);
                if (kb.digit2Key.wasPressedThisFrame) ApplyShot(1);
                if (kb.digit3Key.wasPressedThisFrame) ApplyShot(2);
                if (kb.digit4Key.wasPressedThisFrame) ApplyShot(3);
            }

            // Gusty wind: slow changes of strength and direction from Perlin noise.
            float t = Time.time;
            float gust = 1f + gustiness * (2f * Mathf.PerlinNoise(t * 0.35f, 0.3f) - 1f);
            float swirl = gustiness * (Mathf.PerlinNoise(0.7f, t * 0.25f) - 0.5f);
            Vector3 w = Quaternion.Euler(0f, swirl * 40f, 0f) * wind * gust;
            world.parameters.wind = windOn ? w : Vector3.zero;

            if (world.Solver != null)
                smoothMs = Mathf.Lerp(smoothMs, (float)world.Solver.LastStepMilliseconds, 0.1f);
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

        private GUIStyle style, title;

        private void OnGUI()
        {
            if (!showText || world == null || world.Solver == null) return;
            if (style == null)
            {
                style = new GUIStyle(GUI.skin.label) { fontSize = 15, richText = true };
                style.normal.textColor = new Color(0.92f, 0.94f, 0.96f);
                title = new GUIStyle(style) { fontSize = 20, fontStyle = FontStyle.Bold };
            }
            var p = world.parameters;
            int torn = 0, splits = 0;
            foreach (ParticleCloth c in new[] { flag, curtain, sheet })
            {
                if (c == null || c.Tearing == null) continue;
                torn += c.Tearing.TornEdgeCount;
                splits += c.Tearing.SplitCount;
            }
            float stretch = flag != null && flag.Cloth != null ? flag.MaxStretch(world.Solver) * 100f : 0f;
            string mode = p.gaussSeidelDistance ? "Gauss-Seidel" : "Jacobi (paper)";

            GUI.color = new Color(0f, 0f, 0f, 0.45f);
            GUI.DrawTexture(new Rect(12, 12, 520, 168), Texture2D.whiteTexture);
            GUI.color = Color.white;
            GUI.Label(new Rect(24, 18, 500, 28), "GamiX Cloth  ·  paper §8", title);
            GUI.Label(new Rect(24, 50, 500, 22),
                      $"Distance constraints: <b>{mode}</b>   ·   flag max stretch: <b>{stretch:F1} %</b>", style);
            GUI.Label(new Rect(24, 72, 500, 22),
                      $"Wind: <b>{(windOn ? p.wind.magnitude.ToString("F1") + " m/s" : "off")}</b>   ·   torn edges: <b>{torn}</b>   ·   split vertices: <b>{splits}</b>", style);
            GUI.Label(new Rect(24, 94, 500, 22),
                      $"Particles: <b>{world.Solver.ActiveCount}</b>   ·   solver: <b>{smoothMs:F1} ms</b>{(cutter != null && cutter.Cutting ? "   ·   <color=#c8ff00><b>CUTTING</b></color>" : "")}", style);
            GUI.Label(new Rect(24, 122, 500, 22), "Left drag: pull   ·   hold <b>C</b>: cut   ·   <b>T</b>: rip the curtain   ·   <b>J</b>: solver   ·   <b>W</b>: wind", style);
            GUI.Label(new Rect(24, 144, 500, 22), "<b>1-4</b>: camera   ·   <b>R</b>: reset   ·   <b>H</b>: hide text   ·   Space: impulse   ·   P: pause", style);
        }
    }
}
