using System.Collections.Generic;
using System.IO;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;

namespace GamiX.Physics.Tests
{
    /// <summary>
    /// Project rule: no Unity physics anywhere in Assets (library and every simulation).
    /// Fails with the file and line of each violation.
    /// </summary>
    public class ProjectRulesTests
    {
        private static readonly Regex[] Forbidden =
        {
            new Regex(@"\bRigidbody(2D)?\b"),
            new Regex(@"(?<![\w.])Physics(2D)?\.(?!Components|Unified|Geometry|Collision|RigidBodies|Rendering|Integration|MassSpring|Demos|Tests|Editor)\w+"),
            new Regex(@"\bUnityEngine\.Physics(2D)?\b"),
            new Regex(@"AddComponent<\s*(UnityEngine\.)?(Box|Sphere|Capsule|Mesh|Wheel|Terrain|Polygon|Edge|Circle|Composite|Tilemap)?Collider(2D)?\s*>"),
            new Regex(@"\b(Hinge|Fixed|Spring|Configurable|Character|Distance|Slider|Wheel|Relative|Target|Friction)Joint(2D)?\b"),
            new Regex(@"\bCharacterController\b"),
            new Regex(@"\bOn(Collision|Trigger)(Enter|Stay|Exit)(2D)?\s*\("),
            new Regex(@"\bPhysicMaterial\b|\bPhysicsMaterial(2D)?\b"),
        };

        [Test]
        public void NoUnityPhysicsInAssets()
        {
            var violations = new List<string>();
            string root = Application.dataPath;
            foreach (string file in Directory.GetFiles(root, "*.cs", SearchOption.AllDirectories))
            {
                if (file.Replace('\\', '/').EndsWith("ProjectRulesTests.cs")) continue;
                string[] lines = File.ReadAllLines(file);
                for (int i = 0; i < lines.Length; i++)
                {
                    string line = lines[i];
                    string code = line.Contains("//") ? line.Substring(0, line.IndexOf("//")) : line;
                    foreach (Regex r in Forbidden)
                    {
                        if (r.IsMatch(code))
                            violations.Add($"{file.Substring(root.Length - "Assets".Length)}:{i + 1}: {line.Trim()}");
                    }
                }
            }
            Assert.IsEmpty(violations, "Unity physics is not allowed:\n" + string.Join("\n", violations));
        }
    }
}
