using GamiX.Physics.Components;
using GamiX.Physics.Rendering;
using UnityEngine;

namespace GamiX.Physics.Demos
{
    /// <summary>Helpers to build the demo scenes from code (no Unity physics components are kept).</summary>
    public static class DemoUtils
    {
        /// <summary>Creates a primitive for display only: its Unity Collider is removed.</summary>
        public static GameObject Visual(PrimitiveType type, string name, Vector3 position, Vector3 scale, Color color, Transform parent = null)
        {
            GameObject go = GameObject.CreatePrimitive(type);
            go.name = name;
            Collider c = go.GetComponent<Collider>();
            if (c != null) Object.Destroy(c);
            go.transform.SetParent(parent, false);
            go.transform.position = position;
            go.transform.localScale = scale;
            go.GetComponent<MeshRenderer>().sharedMaterial = RenderUtils.CreateLitMaterial(color);
            return go;
        }

        public static UnifiedWorld CreateUnifiedWorld(string name = "Unified World")
        {
            var go = new GameObject(name);
            go.SetActive(false); // configure before Awake
            UnifiedWorld world = go.AddComponent<UnifiedWorld>();
            go.AddComponent<ParticleRenderer>();
            return world;
        }

        public static ParticleCollider Collider(UnifiedWorld world, string name, ColliderShape shape, Vector3 position, Vector3 size,
                                                bool inverted = false, Quaternion? rotation = null)
        {
            var go = new GameObject(name);
            go.transform.SetParent(world.transform, false);
            go.transform.SetPositionAndRotation(position, rotation ?? Quaternion.identity);
            ParticleCollider c = go.AddComponent<ParticleCollider>();
            c.shape = shape;
            c.size = size;
            c.inverted = inverted;
            return c;
        }

        /// <summary>Ground: a particle collider plane plus a visual quad.</summary>
        public static void Ground(UnifiedWorld world, float size = 20f)
        {
            Collider(world, "Ground Collider", ColliderShape.Plane, Vector3.zero, Vector3.one);
            Visual(PrimitiveType.Plane, "Ground", Vector3.zero, new Vector3(size / 10f, 1f, size / 10f), new Color(0.35f, 0.37f, 0.4f));
        }

        /// <summary>Open glass-like tank: inverted box collider plus thin walls for display.</summary>
        public static void Tank(UnifiedWorld world, Vector3 center, Vector3 size)
        {
            Collider(world, "Tank Collider", ColliderShape.Box, center, size, true);
            var col = new Color(0.6f, 0.7f, 0.8f);
            float t = 0.03f;
            Vector3 h = size * 0.5f;
            Visual(PrimitiveType.Cube, "Wall -X", center + new Vector3(-h.x - t, 0, 0), new Vector3(t, size.y, size.z), col);
            Visual(PrimitiveType.Cube, "Wall +X", center + new Vector3(h.x + t, 0, 0), new Vector3(t, size.y, size.z), col);
            // Back wall only: the demo cameras look from -Z, so the front stays open.
            Visual(PrimitiveType.Cube, "Wall +Z", center + new Vector3(0, 0, h.z + t), new Vector3(size.x, size.y, t), col);
        }

        public static T Body<T>(UnifiedWorld world, string name, Vector3 position, Color color) where T : ParticleBody
        {
            var go = new GameObject(name);
            go.SetActive(false);
            go.transform.SetParent(world.transform, false);
            go.transform.position = position;
            T body = go.AddComponent<T>();
            body.color = color;
            return body;
        }

        public static T BodyWithMesh<T>(UnifiedWorld world, PrimitiveType type, string name, Vector3 position, Vector3 scale, Color color)
            where T : ParticleBody
        {
            GameObject go = Visual(type, name, position, scale, color, world.transform);
            go.SetActive(false);
            T body = go.AddComponent<T>();
            body.color = color;
            body.renderParticles = false;
            return body;
        }
    }
}
