using System.Collections;
using System.Collections.Generic;
using System.IO;
using GamiX.Physics.Rendering;
using GamiX.Physics.Unified;
using UnityEngine;
using UnityEngine.UI;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Two short clips for the cloth slides, run by our solver:
    ///   Links: a small grid, its stretch, shear and bend links appear, then it is released and hangs (section 8).
    ///   Tearing: a tartan cloth with a notch is pulled apart; the crack runs and the split vertices are shown.
    /// With <see cref="record"/> on, the clip runs at a fixed frame rate and every frame is saved as a JPG.
    /// </summary>
    public sealed class ClothSlideClips : MonoBehaviour
    {
        public enum Clip { Links, Tearing }

        public Clip clip = Clip.Links;
        public bool loop = true;

        [Header("Recording")]
        public bool record;
        public string recordFolder = "";
        public int recordWidth = 1000;
        public int recordHeight = 800;
        public int recordFps = 30;

        private static readonly Color Background = Hex(0xEFE4DA);
        private static readonly Color Ink = Hex(0x3B2A22);
        private static readonly Color Soft = Hex(0x9A816B);
        private static readonly Color Stretch = Hex(0x7A604E);
        private static readonly Color Shear = Hex(0x0E93A3);
        private static readonly Color Bend = Hex(0xB02A2A);
        private static readonly Color Particle = Hex(0x0E93A3);
        private static readonly Color Pinned = Hex(0x5A4A40);

        private Camera cam;
        private Canvas canvas;
        private Font font;
        private Material lineMaterial, clothMaterial;
        private Mesh sphere, clothMesh;
        private readonly List<LineRenderer> lines = new List<LineRenderer>();
        private readonly List<Text> texts = new List<Text>();
        private int linesUsed, textsUsed;
        private float time;
        private int steps;

        private UnifiedSolver solver;
        private ClothHandle cloth;
        private ClothTearing tearing;
        private readonly List<int> triangles = new List<int>();
        private readonly List<(int copy, int source)> splits = new List<(int, int)>();
        private int resU, resV;
        private bool notched;
        private Vector3[] grip;

        private RenderTexture target;
        private Texture2D readback;
        private int frame;
        private bool recording;

        private float Duration => clip == Clip.Links ? 12f : 11f;

        private void Awake()
        {
            cam = Camera.main;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Background;
            cam.fieldOfView = 30f;
            cam.nearClipPlane = 0.1f;
            cam.transform.SetPositionAndRotation(new Vector3(0f, 0f, -10f), Quaternion.identity);
            lineMaterial = new Material(Shader.Find("Sprites/Default"));
            BuildSphere();
            BuildCanvas();
        }

        private void Start()
        {
            if (record) StartRecording();
            Restart();
        }

        private void Restart()
        {
            time = 0f;
            steps = 0;
            splits.Clear();
            notched = false;
            if (clip == Clip.Links)
            {
                // Seen from above and in front: the cloth lies flat, then sags between its corners.
                cam.transform.position = new Vector3(0f, 3.4f, -6.2f);
                cam.transform.LookAt(new Vector3(0f, 0.1f, 0.2f));
                BuildLinks();
            }
            else
            {
                cam.transform.SetPositionAndRotation(new Vector3(0f, 0f, -10f), Quaternion.identity);
                BuildTearing();
            }
        }

        // ================================================================ scenes

        private void BuildLinks()
        {
            var p = new SolverParams { substeps = 4, iterations = 8, radius = 0.05f, damping = 0.4f, multithreaded = false };
            solver = new UnifiedSolver(p);
            resU = 6;
            resV = 5;
            cloth = ClothBuilder.CreateCloth(solver, new Vector3(-1.5f, 0.9f, -1.1f), new Vector3(3f, 0f, 0f), new Vector3(0f, 0f, 2.2f),
                resU, resV, 0.05f, 1f, 0.8f, 0.3f, (u, v) => v == resV - 1 && (u == 0 || u == resU - 1));
        }

        private void BuildTearing()
        {
            var p = new SolverParams { substeps = 4, iterations = 6, radius = 0.02f, damping = 0.2f, multithreaded = true };
            solver = new UnifiedSolver(p);
            resU = 22;
            resV = 17;
            int first = solver.DistanceConstraints.Count;
            cloth = ClothBuilder.CreateCloth(solver, new Vector3(-1.6f, 1.3f, 0f), new Vector3(3.2f, 0f, 0f), new Vector3(0f, -2.4f, 0f),
                resU, resV, 0.02f, 1f, 0.8f, 0.3f, (u, v) => v == 0);
            tearing = new ClothTearing(solver, cloth, first, solver.DistanceConstraints.Count - first) { tearRatio = 1.25f, maxTearsPerUpdate = 3 };
            tearing.ParticleSplit += (copy, source) => splits.Add((copy, source));
            // The bottom edge is gripped: its left half and its right half will be pulled apart.
            grip = new Vector3[resU];
            for (int u = 0; u < resU; u++)
            {
                grip[u] = solver.positions[Corner(u)];
                if (u != resU / 2) solver.SetInverseMass(Corner(u), 0f);   // the notch column stays free
            }
            clothMaterial = RenderUtils.CreateLitMaterial(Color.white);
            clothMaterial.enableInstancing = false;
            if (clothMaterial.HasProperty("_BaseMap")) clothMaterial.SetTexture("_BaseMap", ClothTextures.Tartan());
            if (clothMaterial.HasProperty("_Cull")) clothMaterial.SetFloat("_Cull", 0f);
            if (clothMaterial.HasProperty("_Smoothness")) clothMaterial.SetFloat("_Smoothness", 0.15f);
            clothMaterial.doubleSidedGI = true;
            clothMesh = new Mesh { name = "Slide Cloth" };
            clothMesh.MarkDynamic();
        }

        private int Corner(int u) => cloth.particles[(resV - 1) * resU + u];

        // ================================================================ timeline

        private void Update()
        {
            time += Time.deltaTime;
            if (time >= Duration)
            {
                if (recording) FinishRecording();
                else if (loop) Restart();
                time = Mathf.Min(time, Duration);
            }

            linesUsed = textsUsed = 0;
            if (clip == Clip.Links) DrawLinks(time);
            else DrawTearing(time);
            Label("GamiX engine  ·  running in Unity", new Vector2(0.97f, 0.035f), 22, Soft, TextAnchor.LowerRight);
            for (int i = linesUsed; i < lines.Count; i++) lines[i].enabled = false;
            for (int i = textsUsed; i < texts.Count; i++) texts[i].enabled = false;
        }

        private void StepTo(float t, float start)
        {
            int wanted = Mathf.Max(0, Mathf.FloorToInt((t - start) * 60f));
            while (steps < wanted)
            {
                if (tearing != null) tearing.Update();
                BeforeStep();
                solver.Step(1f / 60f);
                steps++;
            }
        }

        private void BeforeStep()
        {
            if (clip != Clip.Tearing) return;
            // Grips: the two halves of the bottom edge move apart; the stress gathers at the tip of the notch.
            float t = steps / 60f + 1.6f;
            float pull = 0.7f * Ease(1.6f, 6.5f, t);
            for (int u = 0; u < resU; u++)
            {
                if (u == resU / 2) continue;
                float side = u < resU / 2 ? -1f : 1f;
                solver.MovePinnedParticle(Corner(u), grip[u] + new Vector3(side * pull, -0.1f * pull, 0f));
            }
        }

        // ---------------------------------------------------------------- links

        private void DrawLinks(float t)
        {
            float release = 7.6f;
            if (t >= release) StepTo(t, release);
            Vector3[] x = solver.positions;
            var rules = solver.DistanceConstraints;
            int[] shown = new int[3];
            float[] start = { 1.0f, 3.0f, 5.0f };
            int[] total = new int[3];
            for (int k = 0; k < rules.Count; k++) total[rules.items[k].kind]++;
            for (int k = 0; k < rules.Count; k++)
            {
                DistanceConstraint r = rules.items[k];
                int kind = r.kind;
                float reveal = Mathf.Clamp01((t - start[kind]) / 1.6f);
                if (shown[kind]++ >= Mathf.CeilToInt(reveal * total[kind])) continue;
                Vector3 a = x[r.i], b = x[r.j];
                if (kind == 0) Line(a, b, Stretch, 0.05f);
                else if (kind == 1) Dashed(a, b, Shear, 0.035f, 7);
                else Arc(a, b, Bend, 0.03f);
            }
            float dot = t < 0.2f ? 0f : Mathf.Min(1f, (t - 0.2f) / 0.6f);
            for (int k = 0; k < cloth.particles.Length; k++)
            {
                int i = cloth.particles[k];
                Ball(x[i], solver.invMass[i] == 0f ? Pinned : Particle, 0.2f * Ease(0f, 1f, dot * 1.2f - k * 0.008f));
            }

            Legend(0.18f, 0.12f, Stretch, "stretch", t > start[0], 0);
            Legend(0.43f, 0.12f, Shear, "shear", t > start[1], 1);
            Legend(0.66f, 0.12f, Bend, "bend", t > start[2], 2);
            if (t > release - 0.3f)
                Label(t < release + 0.4f ? "release: gravity on" : "only distance rules hold it", new Vector2(0.5f, 0.93f), 30, Ink, TextAnchor.MiddleCenter);
        }

        private void Legend(float vx, float vy, Color c, string text, bool on, int kind)
        {
            if (!on) return;
            Vector3 a = cam.ViewportToWorldPoint(new Vector3(vx, vy, 6f)), b = cam.ViewportToWorldPoint(new Vector3(vx + 0.06f, vy, 6f));
            if (kind == 0) Line(a, b, c, 0.05f);
            else if (kind == 1) Dashed(a, b, c, 0.035f, 3);
            else Arc(a, b, c, 0.03f);
            Label(text, new Vector2(vx + 0.075f, vy), 30, Ink, TextAnchor.MiddleLeft);
        }

        // ---------------------------------------------------------------- tearing

        private void DrawTearing(float t)
        {
            if (t >= 1.6f) StepTo(t, 1.6f);
            if (!notched && t >= 1.0f)
            {
                notched = true;
                // The notch: a short cut up from the middle of the bottom edge.
                Vector3 a = solver.positions[Corner(resU / 2)];
                Vector3 b = solver.positions[cloth.particles[(resV - 3) * resU + resU / 2]];
                tearing.Cut(a, b, 0.05f);
                tearing.Update();
            }

            // Mesh from the tearing topology (vertices = particles, including the copies made by the splits).
            int n = tearing.MeshVertexCount;
            var verts = new Vector3[n];
            var uv = new Vector2[n];
            for (int k = 0; k < n; k++)
            {
                verts[k] = solver.positions[tearing.MeshParticle(k)];
                uv[k] = tearing.MeshUv(k);
            }
            tearing.GetMeshTriangles(triangles);
            for (int k = 0; k + 2 < triangles.Count; k += 3)   // face the camera (and the light)
                (triangles[k + 1], triangles[k + 2]) = (triangles[k + 2], triangles[k + 1]);
            clothMesh.Clear();
            clothMesh.vertices = verts;
            clothMesh.uv = uv;
            clothMesh.SetTriangles(triangles, 0);
            clothMesh.RecalculateNormals();
            clothMesh.RecalculateBounds();
            Graphics.DrawMesh(clothMesh, Matrix4x4.identity, clothMaterial, 0);

            // Hands and the split vertices: each copy (red) next to the particle it was split from (teal).
            Vector3 z = new Vector3(0f, -0.04f, -0.1f);
            Line(solver.positions[Corner(0)] + z, solver.positions[Corner(resU / 2 - 1)] + z, Pinned, 0.09f);
            Line(solver.positions[Corner(resU / 2 + 1)] + z, solver.positions[Corner(resU - 1)] + z, Pinned, 0.09f);
            foreach (var (copy, source) in splits)
            {
                Ball(solver.positions[source] + new Vector3(0f, 0f, -0.08f), Particle, 0.085f);
                Ball(solver.positions[copy] + new Vector3(0f, 0f, -0.08f), Bend, 0.085f);
            }

            if (t < 1.0f)
                Label("a notch, then the two halves of the bottom are pulled apart", new Vector2(0.5f, 0.93f), 28, Ink, TextAnchor.MiddleCenter);
            else
                Label($"torn edges <b>{tearing.TornEdgeCount}</b>   ·   split vertices <b>{tearing.SplitCount}</b>",
                    new Vector2(0.5f, 0.93f), 30, Ink, TextAnchor.MiddleCenter);
            Label("<color=#0E93A3>●</color> original particle    <color=#B02A2A>●</color> its copy on the other side",
                new Vector2(0.5f, 0.085f), 26, Soft, TextAnchor.MiddleCenter);
        }

        // ================================================================ drawing

        private void Ball(Vector3 p, Color color, float size)
        {
            if (size <= 0.001f) return;
            Graphics.DrawMesh(sphere, RenderUtils.ScaleTranslation(p, size), RenderUtils.GetSharedMaterial(color), 0);
        }

        private void Line(Vector3 a, Vector3 b, Color color, float width)
        {
            LineRenderer lr = NextLine();
            lr.positionCount = 2;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
            lr.widthMultiplier = width;
            lr.startColor = lr.endColor = color;
        }

        private void Dashed(Vector3 a, Vector3 b, Color color, float width, int dashes)
        {
            for (int k = 0; k < dashes; k++)
                Line(Vector3.Lerp(a, b, (k + 0.15f) / dashes), Vector3.Lerp(a, b, (k + 0.7f) / dashes), color, width);
        }

        /// <summary>Bend links span two particles: drawn as a shallow arc so they do not hide the stretch links.</summary>
        private void Arc(Vector3 a, Vector3 b, Color color, float width)
        {
            Vector3 d = b - a;
            // Bulge upward (out of a horizontal cloth); sideways for a vertical link.
            Vector3 dir = d.normalized;
            Vector3 n = Mathf.Abs(dir.y) > 0.9f ? new Vector3(-dir.y, dir.x, 0f) : Vector3.up;
            Vector3 side = n * (d.magnitude * 0.16f);
            const int seg = 10;
            for (int k = 0; k < seg; k += 2)
            {
                float u0 = (float)k / seg, u1 = (k + 1f) / seg;
                Line(a + d * u0 + side * (4f * u0 * (1f - u0)), a + d * u1 + side * (4f * u1 * (1f - u1)), color, width);
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
                lr.numCapVertices = 3;
                lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                lr.receiveShadows = false;
                lines.Add(lr);
            }
            LineRenderer l = lines[linesUsed++];
            l.enabled = true;
            return l;
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
            rt.pivot = anchor == TextAnchor.MiddleLeft ? new Vector2(0f, 0.5f) : anchor == TextAnchor.LowerRight ? new Vector2(1f, 0f) : new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = Vector2.zero;
            rt.sizeDelta = new Vector2(10f, 10f);
        }

        private void BuildCanvas()
        {
            font = Font.CreateDynamicFontFromOSFont(new[] { "Segoe UI", "Arial" }, 48);
            var go = new GameObject("Clip Canvas");
            go.transform.SetParent(transform, false);
            canvas = go.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceCamera;
            canvas.worldCamera = cam;
            canvas.planeDistance = 1f;
            var scaler = go.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1000f, 800f);
            scaler.matchWidthOrHeight = 1f;
        }

        private void BuildSphere()
        {
            ParticleFactory.Icosphere(3, out List<Vector3> v, out List<int> tris);
            var verts = new Vector3[v.Count];
            var normals = new Vector3[v.Count];
            for (int i = 0; i < v.Count; i++)
            {
                normals[i] = v[i].normalized;
                verts[i] = normals[i] * 0.5f;
            }
            sphere = new Mesh { name = "Clip Sphere", vertices = verts, normals = normals, triangles = tris.ToArray() };
            sphere.bounds = new Bounds(Vector3.zero, Vector3.one);
        }

        private static float Ease(float a, float b, float t)
        {
            float u = Mathf.Clamp01((t - a) / (b - a));
            return u * u * (3f - 2f * u);
        }

        private static Color Hex(int rgb) =>
            new Color(((rgb >> 16) & 255) / 255f, ((rgb >> 8) & 255) / 255f, (rgb & 255) / 255f);

        // ================================================================ recording

        private void StartRecording()
        {
            Directory.CreateDirectory(recordFolder);
            foreach (string f in Directory.GetFiles(recordFolder, "*.jpg")) File.Delete(f);
            string done = Path.Combine(recordFolder, "done.txt");
            if (File.Exists(done)) File.Delete(done);
            target = new RenderTexture(recordWidth, recordHeight, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 8 };
            readback = new Texture2D(recordWidth, recordHeight, TextureFormat.RGB24, false);
            cam.targetTexture = target;
            Time.captureFramerate = recordFps;
            Application.runInBackground = true;
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
        }
    }
}
