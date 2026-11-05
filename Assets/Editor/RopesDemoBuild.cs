using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class RopesDemoBuild
{
    public static void BuildWindows()
    {
        string output = Path.GetFullPath("Builds/RopesDemo/GamiX Ropes Demo.exe");
        var options = new BuildPlayerOptions
        {
            scenes = new[] { "Assets/Simulations/Oubay/Ropes/Scenes/09_Ropes.unity" },
            locationPathName = output,
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.None
        };
        BuildReport report = BuildPipeline.BuildPlayer(options);
        Debug.Log("Ropes demo build result: " + report.summary.result);
        Debug.Log("Ropes demo build output: " + output);
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception("Ropes demo build failed: " + report.summary.result);
    }
}
