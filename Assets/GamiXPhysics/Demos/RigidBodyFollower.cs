using GamiX.Physics.RigidBodies;
using UnityEngine;

namespace GamiX.Physics.Demos
{
    /// <summary>Copies a RigidBody pose into a Transform (for bodies created from code).</summary>
    public sealed class RigidBodyFollower : MonoBehaviour
    {
        [System.NonSerialized] public RigidBody body;

        private void LateUpdate()
        {
            if (body != null)
                transform.SetPositionAndRotation(body.position, body.orientation.ToUnity());
        }
    }
}
