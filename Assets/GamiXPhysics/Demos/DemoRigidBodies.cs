using GamiX.Physics.Components;
using GamiX.Physics.Geometry;
using GamiX.Physics.RigidBodies;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Physics.Demos
{
    /// <summary>
    /// Classic rigid bodies (chapter 2 part 2 and chapter 4): box stacks and a pyramid resting on the ground,
    /// spheres with restitution, and a T-handle spinning in zero gravity around its intermediate axis
    /// (Dzhanibekov effect, which only appears with a correct inertia tensor and angular momentum).
    /// Left click: shoot a ball from the camera. Right click on a body: push it (force at a point -> torque).
    /// </summary>
    public sealed class DemoRigidBodies : MonoBehaviour
    {
        private RigidWorld world;
        private Camera cam;

        private void Awake()
        {
            var go = new GameObject("Rigid World");
            world = go.AddComponent<RigidWorld>();
            world.substeps = 2;
            world.solverIterations = 12;

            Plane(new Vector3(0f, 0f, 0f));
            DemoUtils.Visual(PrimitiveType.Plane, "Ground", Vector3.zero, new Vector3(3f, 1f, 3f), new Color(0.35f, 0.37f, 0.4f));

            // Pyramid of boxes.
            int rows = 6;
            for (int r = 0; r < rows; r++)
            for (int c = 0; c < rows - r; c++)
                Box(new Vector3(-3f + c * 0.52f + r * 0.26f, 0.25f + r * 0.5f, 0f), Vector3.one * 0.5f, Color.HSVToRGB(0.1f * r, 0.5f, 0.9f));

            // Tower.
            for (int i = 0; i < 8; i++)
                Box(new Vector3(1.5f, 0.2f + i * 0.4f, 0f), new Vector3(0.6f, 0.4f, 0.6f), new Color(0.9f, 0.9f, 0.9f));

            // Bouncing spheres with different restitution.
            for (int i = 0; i < 4; i++)
            {
                RigidBodyComponent s = Sphere(new Vector3(3f + i * 0.7f, 3f, -1f), 0.5f, Color.HSVToRGB(0.6f, 0.2f * (i + 1), 1f));
                s.restitution = 0.2f + 0.25f * i;
            }

            // Spinning T-handle, no gravity: flips periodically around its intermediate axis.
            RigidBodyComponent t = Box(new Vector3(0f, 4f, -3f), new Vector3(0.15f, 0.6f, 1.6f), new Color(1f, 0.5f, 0.1f));
            t.useGravity = false;
            t.angularDamping = 0f;
            t.linearDamping = 0f;
            t.initialAngularVelocity = new Vector3(0.02f, 6f, 0f);
        }

        private void Start()
        {
            cam = Camera.main;
        }

        private RigidBodyComponent Box(Vector3 pos, Vector3 size, Color color)
        {
            GameObject go = DemoUtils.Visual(PrimitiveType.Cube, "Box", pos, size, color, world.transform);
            RigidBodyComponent rb = go.AddComponent<RigidBodyComponent>();
            rb.shape = RigidShapeType.Box;
            rb.density = 200f;
            return rb;
        }

        private RigidBodyComponent Sphere(Vector3 pos, float diameter, Color color)
        {
            GameObject go = DemoUtils.Visual(PrimitiveType.Sphere, "Sphere", pos, Vector3.one * diameter, color, world.transform);
            RigidBodyComponent rb = go.AddComponent<RigidBodyComponent>();
            rb.shape = RigidShapeType.Sphere;
            rb.density = 300f;
            return rb;
        }

        private void Plane(Vector3 pos)
        {
            var go = new GameObject("Ground Plane");
            go.transform.SetParent(world.transform, false);
            go.transform.position = pos;
            RigidBodyComponent rb = go.AddComponent<RigidBodyComponent>();
            rb.shape = RigidShapeType.Plane;
            rb.mode = RigidBodyMode.Static;
            rb.friction = 0.6f;
        }

        private void Update()
        {
            Mouse mouse = Mouse.current;
            if (mouse == null || cam == null || world.World == null) return;
            Vector2 mp = mouse.position.ReadValue();
            UnityEngine.Ray r = cam.ScreenPointToRay(new Vector3(mp.x, mp.y, 0f));

            if (mouse.leftButton.wasPressedThisFrame)
            {
                var ball = new RigidBody(new SphereShape(0.2f), 5f, r.origin + r.direction * 1f, Quat.Identity)
                {
                    restitution = 0.4f
                };
                ball.Velocity = r.direction * 20f;
                world.World.AddBody(ball);
                GameObject vis = DemoUtils.Visual(PrimitiveType.Sphere, "Projectile", ball.position, Vector3.one * 0.4f, new Color(0.9f, 0.2f, 0.2f));
                vis.AddComponent<RigidBodyFollower>().body = ball;
            }
            if (mouse.rightButton.wasPressedThisFrame &&
                world.World.Raycast(new Ray3D(r.origin, r.direction), 100f, out RigidBody hitBody, out RayHit hit) && hitBody.IsDynamic)
            {
                hitBody.Wake();
                hitBody.ApplyImpulseAtPoint(r.direction * (3f * hitBody.mass), hit.point);
            }
        }
    }
}
