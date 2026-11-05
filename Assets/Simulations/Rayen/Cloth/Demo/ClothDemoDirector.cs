using System.Collections;
using System.Collections.Generic;
using System.IO;
using GamiX.Physics.Demos;
using UnityEngine;

namespace GamiX.Sim.Cloth
{
    /// <summary>
    /// Scripted demo of the cloth showcase, for the presentation video:
    ///   intro: wide shot, then the camera glides to the flag, the curtain and the sheet, and each one shows its
    ///          particles (the mesh fades into the particles it is made of, like the paper's figure 17);
    ///   values: wide shot with the settings panel, the sliders move one by one (wind, aerodynamics, solver, gravity);
    ///   tear: the camera goes to the curtain and the blade rips it.
    /// The timeline runs on Time.time, so with <see cref="record"/> on (fixed frame rate) every run gives the same
    /// frames. The mesh / particle fades are made by recording twice (forceView 0 and 1) and blending the frames.
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public sealed class ClothDemoDirector : MonoBehaviour
    {
        [Header("Recording")]
        public bool record;
        [Tooltip("Folder for the frames (absolute path).")]
        public string recordFolder = "";
        public int recordFps = 30;
        [Tooltip("-1 = follow the timeline, 0 = always the mesh, 1 = always the particles.")]
        public int forceView = -1;
        [Tooltip("Stop the recording at this time (seconds). Negative = the whole timeline.")]
        public float stopAt = -1f;

        private ClothShowcase show;
        private OrbitCamera orbit;
        private float startSpeed, startAngle, startGusts, startDrag, startLift, startGravity;
        private int startIterations;
        private bool ripped, recording, holding;
        private int frame;
        private StreamWriter alphaLog;

        // ---------------------------------------------------------------- timeline (seconds)

        private struct Pose
        {
            public Vector3 target;
            public float distance, yaw, pitch;
        }

        // Camera: (arrival time, pose index); the camera glides from the previous pose during `Glide` seconds.
        private const float Glide = 1.3f;
        private static readonly (float arrive, int shot)[] CameraPath =
        {
            (0f, 0),      // wide
            (2.5f, 1),    // flag
            (5.9f, 2),    // curtain
            (9.3f, 3),    // sheet
            (12.9f, 0),   // wide, settings panel
            (23.4f, 2),   // curtain: tear
        };

        // Particle view fades: (fade in start, fade out start), each fade lasts `Fade` seconds.
        private const float Fade = 0.5f;
        private static readonly (float fadeIn, float fadeOut)[] ParticleFades = { (2.9f, 4.1f), (6.3f, 7.5f), (9.7f, 10.9f) };

        private const float PanelOn = 12.6f, PanelOff = 22.0f, RipAt = 23.9f;
        public const float Duration = 30f;
        public const float IntroEnd = 11.6f;

        private void Start()
        {
            show = FindAnyObjectByType<ClothShowcase>();
            orbit = Camera.main != null ? Camera.main.GetComponent<OrbitCamera>() : null;
            show.scripted = true;
            show.showText = false;
            startSpeed = show.WindSpeed;
            startAngle = show.WindAngle;
            startGusts = show.gustiness;
            startDrag = show.Aerodynamics.drag;
            startLift = show.Aerodynamics.lift;
            startGravity = -show.World.parameters.gravity.y;
            startIterations = show.World.parameters.iterations;
            if (record)
                StartRecording();
        }

        private void Update()
        {
            float t = Time.time;
            ApplyCamera(t);

            float alpha = ParticleAlpha(t);
            bool particles = forceView == 1 || (forceView == -1 && alpha > 0.5f);
            if (particles != show.ParticleView)
                show.SetParticleView(particles);
            if (recording && alphaLog != null)
                alphaLog.WriteLine(alpha.ToString("0.0000", System.Globalization.CultureInfo.InvariantCulture));

            show.showText = t >= PanelOn && t < PanelOff;
            ApplyValues(t);

            // Strong wind could tear the curtain before its turn: tearing waits for the rip.
            bool hold = t >= PanelOn && t < PanelOff;
            if (hold != holding)
            {
                holding = hold;
                show.HoldTears(hold);
            }

            if (!ripped && t >= RipAt)
            {
                ripped = true;
                show.Rip();
            }

            float end = stopAt > 0f ? Mathf.Min(stopAt, Duration) : Duration;
            if (recording && t >= end)
                FinishRecording();
        }

        private void ApplyCamera(float t)
        {
            if (orbit == null) return;
            int k = 0;
            while (k + 1 < CameraPath.Length && t >= CameraPath[k + 1].arrive - Glide)
                k++;
            Pose to = GetPose(CameraPath[k].shot);
            Pose p = to;
            if (k > 0)
            {
                Pose from = GetPose(CameraPath[k - 1].shot);
                float u = Ease(CameraPath[k].arrive - Glide, CameraPath[k].arrive, t);
                p.target = Vector3.Lerp(from.target, to.target, u);
                p.distance = Mathf.Lerp(from.distance, to.distance, u);
                p.yaw = Mathf.LerpAngle(from.yaw, to.yaw, u);
                p.pitch = Mathf.Lerp(from.pitch, to.pitch, u);
            }
            orbit.target = p.target;
            orbit.distance = p.distance;
            orbit.yaw = p.yaw;
            orbit.pitch = p.pitch;
        }

        private Pose GetPose(int shot)
        {
            show.GetShot(shot, out Vector3 target, out float distance, out float yaw, out float pitch);
            return new Pose { target = target, distance = distance, yaw = yaw, pitch = pitch };
        }

        public static float ParticleAlpha(float t)
        {
            float a = 0f;
            foreach (var (fadeIn, fadeOut) in ParticleFades)
                a = Mathf.Max(a, Ease(fadeIn, fadeIn + Fade, t) * (1f - Ease(fadeOut, fadeOut + Fade, t)));
            return a;
        }

        /// <summary>The settings panel at work: one value at a time, its row highlighted.</summary>
        private void ApplyValues(float t)
        {
            var p = show.World.parameters;
            var aero = show.Aerodynamics;
            string hl = null;

            float speed = startSpeed, angle = startAngle, gusts = startGusts, drag = startDrag, lift = startLift;
            float gravity = startGravity, iterations = startIterations;

            // Just a few values: stronger wind, a change of direction, then lower gravity.
            speed = Step(t, 13.3f, 14.8f, speed, 11f, "Speed", ref hl);
            angle = Step(t, 15.0f, 16.4f, angle, -35f, "Direction", ref hl);
            angle = Step(t, 16.6f, 17.6f, angle, startAngle, "Direction", ref hl);
            gravity = Step(t, 17.8f, 18.8f, gravity, 4f, "Gravity", ref hl);
            gravity = Step(t, 19.8f, 20.6f, gravity, startGravity, "Gravity", ref hl);
            speed = Step(t, 20.6f, 21.4f, speed, startSpeed, "Speed", ref hl);

            show.WindSpeed = speed;
            show.WindAngle = angle;
            show.gustiness = gusts;
            aero.drag = drag;
            aero.lift = lift;
            p.iterations = Mathf.Max(1, Mathf.RoundToInt(iterations));
            p.gravity = new Vector3(0f, -gravity, 0f);
            show.highlight = hl;
        }

        /// <summary>Value of a tween from `value` (before `a`) to `to` (after `b`); sets the highlight while it runs.</summary>
        private static float Step(float t, float a, float b, float value, float to, string label, ref string highlight)
        {
            if (t < a) return value;
            if (t <= b + 0.35f && label != null) highlight = label;
            return Mathf.Lerp(value, to, Ease(a, b, t));
        }

        private static float Ease(float a, float b, float t)
        {
            float u = Mathf.Clamp01((t - a) / (b - a));
            return u * u * (3f - 2f * u);
        }

        // ---------------------------------------------------------------- recording

        private void StartRecording()
        {
            Directory.CreateDirectory(recordFolder);
            foreach (string f in Directory.GetFiles(recordFolder, "*.jpg"))
                File.Delete(f);
            string done = Path.Combine(recordFolder, "done.txt");
            if (File.Exists(done)) File.Delete(done);
            alphaLog = new StreamWriter(Path.Combine(recordFolder, "alpha.txt"));
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
                Texture2D shot = ScreenCapture.CaptureScreenshotAsTexture();
                File.WriteAllBytes(Path.Combine(recordFolder, $"frame_{frame:0000}.jpg"), shot.EncodeToJPG(93));
                Destroy(shot);
                frame++;
            }
        }

        private void FinishRecording()
        {
            recording = false;
            Time.captureFramerate = 0;
            alphaLog?.Close();
            alphaLog = null;
            File.WriteAllText(Path.Combine(recordFolder, "done.txt"), frame.ToString());
        }

        private void OnDestroy()
        {
            alphaLog?.Close();
        }
    }
}
