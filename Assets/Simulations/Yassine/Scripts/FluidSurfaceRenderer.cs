using System.Collections.Generic;
using GamiX.Physics.Components;
using UnityEngine;
using UnityEngine.Rendering;

namespace GamiX.Sim.Yassine
{
    /// <summary>
    /// Draws each fluid phase as a smooth surface (<see cref="FluidSurfaceMesher"/>), rebuilt every frame from the
    /// particle positions. One mesh and one color per phase, so mixing fluids stay distinguishable.
    /// </summary>
    public sealed class FluidSurfaceRenderer : MonoBehaviour
    {
        public UnifiedWorld world;
        [Range(0f, 1f)] public float smoothness = 0.92f;

        private sealed class Layer
        {
            public IEnumerable<int> particles;
            public Mesh mesh;
            public Material material;
            public readonly FluidSurfaceMesher mesher = new FluidSurfaceMesher();
        }

        private readonly List<Layer> layers = new List<Layer>();

        /// <summary>Total surface triangles last frame (for the stats line).</summary>
        public int Triangles { get; private set; }
        public double LastBuildMilliseconds { get; private set; }

        public void AddLayer(IEnumerable<int> particles, Color color)
        {
            var mesh = new Mesh { name = "Fluid surface", indexFormat = IndexFormat.UInt32 };
            mesh.MarkDynamic();
            layers.Add(new Layer { particles = particles, mesh = mesh, material = FluidMaterials.Glossy(color, smoothness) });
        }

        private void LateUpdate()
        {
            if (world == null || world.Solver == null) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();
            int tris = 0;
            foreach (Layer l in layers)
            {
                l.mesher.Build(world.Solver, l.particles);
                l.mesh.Clear();
                if (l.mesher.triangles.Count == 0) continue;
                l.mesh.SetVertices(l.mesher.vertices);
                l.mesh.SetNormals(l.mesher.normals);
                l.mesh.SetTriangles(l.mesher.triangles, 0, true);
                tris += l.mesher.triangles.Count / 3;
                Graphics.RenderMesh(new RenderParams(l.material), l.mesh, 0, Matrix4x4.identity);
            }
            Triangles = tris;
            LastBuildMilliseconds = sw.Elapsed.TotalMilliseconds;
        }

        private void OnDestroy()
        {
            foreach (Layer l in layers)
            {
                Destroy(l.mesh);
                Destroy(l.material);
            }
        }
    }
}
