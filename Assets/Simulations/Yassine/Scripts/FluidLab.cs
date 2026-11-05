using System.Collections.Generic;
using GamiX.Physics;
using GamiX.Physics.Components;
using GamiX.Physics.Demos;
using GamiX.Physics.Unified;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Sim.Yassine
{
    /// <summary>
    /// Yassine's simulation: fluids of the unified particle solver (paper section 7) and their coupling with rigid
    /// bodies. Builds one of the <see cref="FluidScenario"/>s at runtime, draws the water as a surface or as
    /// particles, and shows a control panel with live solver settings and measurements.
    ///
    /// Keys: 1-4 scenario, R reset, V surface/particles, C color mode, B drop a crate, E emitter, H hide the panel.
    /// From the library's ParticleGrabber: left mouse drags a particle, Space gives a radial impulse, P pauses.
    /// </summary>
    [DefaultExecutionOrder(-50)] // before the ParticleGrabber, which is switched off while the mouse is over the panel
    public sealed class FluidLab : MonoBehaviour
    {
        public FluidScenario scenario = FluidScenario.DamBreak;
        public FluidColorMode colorMode = FluidColorMode.Speed;
        [SerializeField] private bool showSurface = true;
        public bool showPanel = true;
        [Tooltip("Max compression drawn as the reference line of the history chart.")]
        public float compressionReference = 0.05f;

        public UnifiedWorld World { get; private set; }
        public ScenarioSetup Setup { get; private set; }
        public float MaxCompression { get; private set; }
        public float CompressionReference => compressionReference;
        public float Fps { get; private set; }
        public readonly List<string> ScenarioLines = new List<string>();

        private GameObject root;
        private FluidParticleRenderer particleRenderer;
        private FluidSurfaceRenderer surfaceRenderer;
        private ParticleGrabber grabber;
        private readonly List<Transform> bodyVisuals = new List<Transform>();
        private readonly FluidControlPanel panel = new FluidControlPanel();
        private float nextSample;
        private int droppedCrates;
        private bool buildRequested;

        private static readonly float[] CrateDensities = { 0.2f, 0.5f, 2f };

        public bool ShowSurface
        {
            get => showSurface;
            set
            {
                showSurface = value;
                ApplyView();
            }
        }

        private void Start()
        {
            Build();
        }

        /// <summary>Rebuilds at the next Update (safe to call from the GUI).</summary>
        public void RequestBuild() => buildRequested = true;

        /// <summary>Destroys the current world and builds the selected scenario.</summary>
        public void Build()
        {
            buildRequested = false;
            if (root != null)
                Destroy(root);
            droppedCrates = 0;
            bodyVisuals.Clear();
            panel.ClearHistory();

            root = new GameObject($"Fluid World ({scenario})");
            root.SetActive(false); // configure before Awake
            World = root.AddComponent<UnifiedWorld>();
            World.parameters = FluidScenarios.Parameters(scenario);
            World.showStats = false; // the panel shows them
            particleRenderer = root.AddComponent<FluidParticleRenderer>();
            particleRenderer.world = World;
            surfaceRenderer = root.AddComponent<FluidSurfaceRenderer>();
            surfaceRenderer.world = World;
            root.SetActive(true); // UnifiedWorld.Awake creates the solver

            Setup = FluidScenarios.Build(World.Solver, scenario);
            foreach (FluidBlock b in Setup.fluids)
            {
                particleRenderer.SetGroupColor(b.group, b.color);
                surfaceRenderer.AddLayer(b.particles, b.color);
            }
            if (Setup.emitter != null)
            {
                particleRenderer.SetGroupColor(Setup.emitter.Group, FluidScenarios.Water);
                surfaceRenderer.AddLayer(Setup.emitter.Particles, FluidScenarios.Water);
            }

            BuildVisuals();
            SetupCamera();
            ApplyView();
            nextSample = 0f;
        }

        private void ApplyView()
        {
            if (particleRenderer == null) return;
            particleRenderer.enabled = !showSurface;
            surfaceRenderer.enabled = showSurface;
            particleRenderer.colorMode = colorMode;
        }

        // ---------------------------------------------------------------- actions (keys and panel buttons)

        public void CycleColors()
        {
            colorMode = (FluidColorMode)(((int)colorMode + 1) % 3);
            ShowSurface = false; // colors only exist on the particle view
        }

        /// <summary>Drops a crate above the tank; its density cycles through floating and sinking values.</summary>
        public void DropCrate()
        {
            float density = CrateDensities[droppedCrates % CrateDensities.Length];
            KinematicBox tank = Setup.tanks[0];
            float x = tank.position.x + 0.4f * ((droppedCrates % 3) - 1) * tank.halfExtents.x;
            float top = tank.position.y + tank.halfExtents.y;
            var size = new Vector3(0.3f, 0.3f, 0.3f);
            RigidGroup g = ParticleFactory.CreateRigidBox(World.Solver, new Vector3(x, top + 0.3f, 0f),
                                                          Quat.FromAxisAngleDeg(new Vector3(1f, 0f, 1f), 25f * droppedCrates), size, density);
            var body = new FloatingBody
            {
                name = $"Crate {density:0.0}", color = Color.HSVToRGB((0.08f + 0.13f * droppedCrates) % 1f, 0.6f, 0.9f),
                size = size, relativeDensity = density, rigid = g
            };
            Setup.bodies.Add(body);
            AddBodyVisual(body);
            droppedCrates++;
        }

        /// <summary>Radial kick from just under the middle of the fluid.</summary>
        public void Splash()
        {
            Vector3 c = VMath.Zero;
            int n = 0;
            foreach (int i in Setup.AllFluid)
            {
                if (!World.Solver.active[i]) continue;
                c += World.Solver.positions[i];
                n++;
            }
            if (n == 0) return;
            c /= n;
            World.Solver.AddRadialImpulse(c - new Vector3(0f, 0.2f, 0f), 0.7f, 5f);
        }

        // ---------------------------------------------------------------- visuals (display only)

        private void BuildVisuals()
        {
            Transform t = root.transform;
            GameObject floor = DemoUtils.Visual(PrimitiveType.Plane, "Floor", Vector3.zero, new Vector3(4f, 1f, 4f), Color.white, t);
            floor.GetComponent<MeshRenderer>().sharedMaterial =
                FluidMaterials.Checker(new Color(0.86f, 0.87f, 0.9f), new Color(0.74f, 0.76f, 0.8f), 40f);

            Material glass = FluidMaterials.Transparent(new Color(0.75f, 0.88f, 1f, 0.12f), 0.95f);
            Material frame = FluidMaterials.Glossy(new Color(0.22f, 0.24f, 0.28f), 0.5f);
            const float wall = 0.02f, bar = 0.04f;
            foreach (KinematicBox tank in Setup.tanks)
            {
                Vector3 c = tank.position, h = tank.halfExtents;
                Panel("Glass -X", c + new Vector3(-h.x - wall, 0f, 0f), new Vector3(wall, 2f * h.y, 2f * h.z), glass, t);
                Panel("Glass +X", c + new Vector3(h.x + wall, 0f, 0f), new Vector3(wall, 2f * h.y, 2f * h.z), glass, t);
                Panel("Glass -Z", c + new Vector3(0f, 0f, -h.z - wall), new Vector3(2f * h.x, 2f * h.y, wall), glass, t);
                Panel("Glass +Z", c + new Vector3(0f, 0f, h.z + wall), new Vector3(2f * h.x, 2f * h.y, wall), glass, t);
                // Frame: vertical posts at the corners and a rim at the top.
                for (int sx = -1; sx <= 1; sx += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                    Panel("Post", c + new Vector3(sx * (h.x + wall), 0f, sz * (h.z + wall)), new Vector3(bar, 2f * h.y, bar), frame, t);
                for (int sz = -1; sz <= 1; sz += 2)
                    Panel("Rim", c + new Vector3(0f, h.y, sz * (h.z + wall)), new Vector3(2f * h.x + bar, bar, bar), frame, t);
                for (int sx = -1; sx <= 1; sx += 2)
                    Panel("Rim", c + new Vector3(sx * (h.x + wall), h.y, 0f), new Vector3(bar, bar, 2f * h.z + bar), frame, t);
            }
            foreach (KinematicBox o in Setup.obstacles)
                Panel("Obstacle", o.position, 2f * o.halfExtents, FluidMaterials.Glossy(new Color(0.45f, 0.46f, 0.5f), 0.3f), t);

            FluidEmitter e = Setup.emitter;
            if (e != null)
            {
                Vector3 dir = VMath.Normalize(e.direction);
                GameObject nozzle = DemoUtils.Visual(PrimitiveType.Cylinder, "Nozzle", e.nozzle - dir * 0.15f,
                                                     new Vector3(2.6f * e.nozzleRadius, 0.15f, 2.6f * e.nozzleRadius), new Color(0.3f, 0.3f, 0.32f), t);
                nozzle.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir);
                Panel("Pipe", e.nozzle - dir * 0.15f + new Vector3(-0.35f, 0f, 0f), new Vector3(0.6f, 0.1f, 0.1f), frame, t);
                Panel("Overflow outlet", e.drainCenter + new Vector3(0.5f * e.drainSize.x + 0.05f, 0f, 0f),
                      new Vector3(0.02f, e.drainSize.y, e.drainSize.z), FluidMaterials.Glossy(new Color(0.06f, 0.06f, 0.08f), 0.2f), t);
            }

            foreach (FloatingBody b in Setup.bodies)
                AddBodyVisual(b);
        }

        private static void Panel(string name, Vector3 position, Vector3 size, Material material, Transform parent)
        {
            GameObject go = DemoUtils.Visual(PrimitiveType.Cube, name, position, size, Color.white, parent);
            go.GetComponent<MeshRenderer>().sharedMaterial = material;
        }

        private void AddBodyVisual(FloatingBody b)
        {
            GameObject go = DemoUtils.Visual(b.sphere ? PrimitiveType.Sphere : PrimitiveType.Cube, b.name, b.rigid.center,
                                             b.sphere ? Vector3.one * b.size.x : b.size, b.color, root.transform);
            go.GetComponent<MeshRenderer>().sharedMaterial = FluidMaterials.Glossy(b.color, 0.55f);
            bodyVisuals.Add(go.transform);
        }

        private void SetupCamera()
        {
            Camera cam = Camera.main;
            if (cam == null) return;
            OrbitCamera orbit = cam.GetComponent<OrbitCamera>();
            if (orbit != null)
            {
                orbit.target = new Vector3(0.35f, 0.55f, 0f); // shifted right: the panel covers the right of the screen
                orbit.distance = scenario == FluidScenario.RayleighTaylor ? 3.4f : 4.6f;
                orbit.yaw = 0f;
                orbit.pitch = 20f;
            }
            // The grabber subscribes to one world: give it the new one.
            if (grabber != null) Destroy(grabber);
            grabber = cam.gameObject.AddComponent<ParticleGrabber>();
            grabber.world = World;
        }

        // ---------------------------------------------------------------- input, sampling, display

        private void Update()
        {
            if (buildRequested)
            {
                Build();
                return;
            }
            Keyboard kb = Keyboard.current;
            if (kb != null)
            {
                FluidScenario before = scenario;
                if (kb.digit1Key.wasPressedThisFrame) scenario = FluidScenario.DamBreak;
                if (kb.digit2Key.wasPressedThisFrame) scenario = FluidScenario.Buoyancy;
                if (kb.digit3Key.wasPressedThisFrame) scenario = FluidScenario.RayleighTaylor;
                if (kb.digit4Key.wasPressedThisFrame) scenario = FluidScenario.Fountain;
                if (scenario != before || kb.rKey.wasPressedThisFrame)
                {
                    Build();
                    return;
                }
                if (kb.vKey.wasPressedThisFrame) ShowSurface = !ShowSurface;
                if (kb.cKey.wasPressedThisFrame) CycleColors();
                if (kb.eKey.wasPressedThisFrame && Setup.emitter != null) Setup.emitter.emitting = !Setup.emitter.emitting;
                if (kb.bKey.wasPressedThisFrame) DropCrate();
                if (kb.hKey.wasPressedThisFrame) showPanel = !showPanel;
            }
            particleRenderer.colorMode = colorMode;

            // No particle grabbing through the panel.
            if (grabber != null && Mouse.current != null)
            {
                Vector2 m = Mouse.current.position.ReadValue();
                var gui = new Vector2(m.x, Screen.height - m.y);
                grabber.enabled = !(showPanel && panel.Rect.Contains(gui));
            }

            float dt = Mathf.Max(1e-4f, Time.unscaledDeltaTime);
            Fps = Fps <= 0f ? 1f / dt : Mathf.Lerp(Fps, 1f / dt, 0.05f);
        }

        private void LateUpdate()
        {
            if (Setup == null) return;
            for (int k = 0; k < bodyVisuals.Count && k < Setup.bodies.Count; k++)
            {
                RigidGroup g = Setup.bodies[k].rigid;
                bodyVisuals[k].SetPositionAndRotation(g.center, g.rotation.ToUnity());
            }
            if (Time.unscaledTime >= nextSample)
            {
                nextSample = Time.unscaledTime + 0.1f;
                Sample();
            }
        }

        /// <summary>10 times per second: the compression metric for the chart and the scenario's readouts.</summary>
        private void Sample()
        {
            UnifiedSolver s = World.Solver;
            FluidStats.DensityRatio(s, Setup.AllFluid, out float avg, out float max);
            MaxCompression = Mathf.Max(0f, max - 1f);
            if (!World.paused)
                panel.PushSample(MaxCompression);

            ScenarioLines.Clear();
            ScenarioLines.Add($"Mean density  <b>{avg:F2}</b> ρ0   ·   t = <b>{s.time:F1}</b> s");
            switch (scenario)
            {
                case FluidScenario.DamBreak:
                    ScenarioLines.Add($"Wave front  x = <b>{FluidStats.FrontX(s, Setup.AllFluid):F2}</b> m");
                    break;
                case FluidScenario.RayleighTaylor:
                    foreach (FluidBlock b in Setup.fluids)
                        ScenarioLines.Add($"{b.name} (×{b.particleMass:0})  mean height <b>{FluidStats.MeanHeight(s, b.particles):F2}</b> m");
                    break;
                case FluidScenario.Fountain:
                    FluidEmitter e = Setup.emitter;
                    ScenarioLines.Add($"Water <b>{e.Alive}</b>  ·  emitted {e.TotalEmitted}  ·  drained {e.TotalDrained}");
                    break;
            }
            if (scenario != FluidScenario.Fountain && Setup.bodies.Count > 0)
            {
                float surface = FluidStats.SurfaceHeight(s, Setup.AllFluid);
                foreach (FloatingBody b in Setup.bodies)
                {
                    float sub = FluidStats.SubmergedFraction(s, b.rigid, surface);
                    bool onFloor = FluidStats.Bottom(s, b.rigid) < 0.03f;
                    string archimedes = b.relativeDensity >= 1f ? "sinks" : $"{100f * b.relativeDensity:0} %";
                    ScenarioLines.Add($"{b.name}  {(onFloor ? "<b>on the floor</b>" : $"<b>{100f * sub:0} %</b> under")}  (Archimedes {archimedes})");
                }
            }
        }

        private void OnGUI()
        {
            if (showPanel && Setup != null)
                panel.Draw(this);
        }
    }
}
