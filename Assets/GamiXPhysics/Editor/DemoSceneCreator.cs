using System;
using System.Collections.Generic;
using System.IO;
using GamiX.Physics.Demos;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace GamiX.Physics.Editor
{
    /// <summary>Menu "GamiX Physics > Create Demo Scenes": builds the demo scenes of the library (course chapters).</summary>
    public static class DemoSceneCreator
    {
        private const string Folder = "Assets/GamiXPhysics/Demos/Scenes";

        private struct DemoInfo
        {
            public string name;
            public Type type;
            public Vector3 target;
            public float distance;
            public bool particles;
        }

        private static readonly DemoInfo[] Demos =
        {
            new DemoInfo { name = "01_RigidBodies", type = typeof(DemoRigidBodies), target = new Vector3(0f, 1.5f, 0f), distance = 12f },
            new DemoInfo { name = "02_BroadPhase", type = typeof(DemoBroadPhase), target = Vector3.zero, distance = 18f },
            new DemoInfo { name = "03_MassSpring", type = typeof(DemoMassSpring), target = new Vector3(1f, 2f, 0f), distance = 11f },
        };

        [MenuItem("GamiX Physics/Create Demo Scenes")]
        public static void CreateAll()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;
            Directory.CreateDirectory(Folder);
            var buildScenes = new List<EditorBuildSettingsScene>();
            foreach (DemoInfo d in Demos)
            {
                string path = $"{Folder}/{d.name}.unity";
                CreateScene(d, path);
                buildScenes.Add(new EditorBuildSettingsScene(path, true));
            }
            EditorBuildSettings.scenes = buildScenes.ToArray();
            AssetDatabase.Refresh();
            EditorSceneManager.OpenScene($"{Folder}/{Demos[0].name}.unity");
            Debug.Log($"GamiX Physics: {Demos.Length} demo scenes created in {Folder}.");
        }

        private static void CreateScene(DemoInfo d, string path)
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            var cam = camGo.AddComponent<Camera>();
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 500f;
            OrbitCamera orbit = camGo.AddComponent<OrbitCamera>();
            orbit.target = d.target;
            orbit.distance = d.distance;
            if (d.particles)
                camGo.AddComponent<ParticleGrabber>();

            var lightGo = new GameObject("Directional Light");
            var light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            light.shadows = LightShadows.Soft;
            lightGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            var demo = new GameObject(d.name.Substring(3));
            demo.AddComponent(d.type);

            EditorSceneManager.SaveScene(scene, path);
        }

        [MenuItem("GamiX Physics/Set Code Optimization To Release")]
        public static void SetRelease()
        {
            UnityEditor.Compilation.CompilationPipeline.codeOptimization = UnityEditor.Compilation.CodeOptimization.Release;
            Debug.Log("GamiX Physics: scripts are now compiled in Release mode (much faster simulation in the editor).");
        }
    }
}
