using GamiX.Physics.Demos;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace GamiX.Sim.Yassine.Editor
{
    /// <summary>Menu "GamiX Physics > Yassine > Create Fluid Scene": writes the simulation's scene in this folder.</summary>
    public static class FluidSceneCreator
    {
        private const string ScenePath = "Assets/Simulations/Yassine/Scenes/Fluids.unity";

        [MenuItem("GamiX Physics/Yassine/Create Fluid Scene")]
        public static void Create()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;
            System.IO.Directory.CreateDirectory("Assets/Simulations/Yassine/Scenes");
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
            Camera cam = camGo.AddComponent<Camera>();
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 200f;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.16f, 0.18f, 0.23f);
            OrbitCamera orbit = camGo.AddComponent<OrbitCamera>();
            orbit.target = new Vector3(0.35f, 0.55f, 0f);
            orbit.distance = 4.6f;
            orbit.yaw = 0f;
            orbit.pitch = 20f;
            // FluidLab adds the ParticleGrabber itself, wired to the world it builds.

            var lightGo = new GameObject("Directional Light");
            Light light = lightGo.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            light.shadows = LightShadows.Soft;
            lightGo.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

            new GameObject("Fluid Lab").AddComponent<FluidLab>();

            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.Refresh();
            Debug.Log($"GamiX Physics: fluid scene created at {ScenePath}. Press Play.");
        }
    }
}
