using UnityEngine;
using UnityEngine.InputSystem;

namespace GamiX.Physics.Demos
{
    /// <summary>Right mouse: orbit. Middle mouse: pan. Wheel: zoom. F: reset.</summary>
    public sealed class OrbitCamera : MonoBehaviour
    {
        public Vector3 target = new Vector3(0f, 1f, 0f);
        public float distance = 6f;
        public float yaw = 30f;
        public float pitch = 25f;
        public float orbitSpeed = 0.25f;
        public float zoomSpeed = 0.002f;

        private Vector3 startTarget;
        private float startDistance, startYaw, startPitch;

        private void Start()
        {
            startTarget = target;
            startDistance = distance;
            startYaw = yaw;
            startPitch = pitch;
            Apply();
        }

        private void LateUpdate()
        {
            Mouse mouse = Mouse.current;
            Keyboard kb = Keyboard.current;
            if (mouse != null)
            {
                Vector2 delta = mouse.delta.ReadValue();
                if (mouse.rightButton.isPressed)
                {
                    yaw += delta.x * orbitSpeed;
                    pitch = Mathf.Clamp(pitch - delta.y * orbitSpeed, -85f, 85f);
                }
                if (mouse.middleButton.isPressed)
                {
                    float k = distance * 0.0015f;
                    target -= transform.right * (delta.x * k) + transform.up * (delta.y * k);
                }
                float scroll = mouse.scroll.ReadValue().y;
                if (Mathf.Abs(scroll) > 0f)
                    distance = Mathf.Clamp(distance * (1f - scroll * zoomSpeed), 0.5f, 100f);
            }
            if (kb != null && kb.fKey.wasPressedThisFrame)
            {
                target = startTarget;
                distance = startDistance;
                yaw = startYaw;
                pitch = startPitch;
            }
            Apply();
        }

        private void Apply()
        {
            Quaternion rot = Quaternion.Euler(pitch, yaw, 0f);
            transform.SetPositionAndRotation(target - rot * Vector3.forward * distance, rot);
        }
    }
}
