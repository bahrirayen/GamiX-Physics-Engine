using UnityEngine;

namespace GamiX.Physics.RigidBodies
{
    /// <summary>
    /// Rigid body following Baraff (SIGGRAPH 97) as presented in chapter 2 part 2.
    /// State X(t) = (x, q, P, L): center of mass, orientation (unit quaternion instead of the 3x3 matrix R, so no
    /// Gram-Schmidt drift), linear momentum P and angular momentum L.
    /// Derived quantities (slide 40): v = P / M, I^-1(t) = R I_body^-1 R^T, w = I^-1(t) L.
    /// </summary>
    public sealed class RigidBody
    {
        public RigidShape shape;

        // ---- Mass properties ----
        public float mass;
        public float inverseMass;
        public Mat3 inertiaBody;
        public Mat3 inverseInertiaBody;

        // ---- State X(t) ----
        public Vector3 position;
        public Quat orientation = Quat.Identity;
        public Vector3 linearMomentum;
        public Vector3 angularMomentum;

        // ---- Accumulators (cleared every step) ----
        public Vector3 force;
        public Vector3 torque;

        // ---- Material and options ----
        public float restitution = 0.3f;
        public float friction = 0.5f;
        public float linearDamping = 0.01f;
        public float angularDamping = 0.02f;
        public bool useGravity = true;

        /// <summary>Kinematic bodies have infinite mass and are moved by script (velocities below are used).</summary>
        public bool isKinematic;
        public Vector3 kinematicVelocity;
        public Vector3 kinematicAngularVelocity;

        /// <summary>Bit masks: two bodies collide when (a.layer &amp; b.collisionMask) != 0 and vice versa.</summary>
        public int layer = 1;
        public int collisionMask = ~0;

        public bool isSleeping;
        public float sleepTimer;
        public bool canSleep = true;

        /// <summary>Free slot for the user (e.g. the MonoBehaviour that owns the body).</summary>
        public object userData;
        public int id = -1;

        // ---- Cached world-space values ----
        private Mat3 rotationMatrix = Mat3.Identity;
        private Mat3 inverseInertiaWorld;

        public RigidBody(RigidShape shape, float mass, Vector3 position, Quat orientation)
        {
            this.shape = shape;
            this.position = position;
            this.orientation = orientation.Normalized;
            SetMass(mass);
        }

        /// <summary>Static (immovable) body: zero inverse mass and zero inverse inertia.</summary>
        public static RigidBody CreateStatic(RigidShape shape, Vector3 position, Quat orientation)
        {
            RigidBody body = new RigidBody(shape, 0f, position, orientation);
            body.canSleep = false;
            return body;
        }

        public bool IsStatic => inverseMass == 0f && !isKinematic;
        public bool IsDynamic => inverseMass > 0f;

        /// <summary>Sets the mass and recomputes I_body from the shape geometry (mass &lt;= 0 means static).</summary>
        public void SetMass(float newMass)
        {
            if (newMass <= 0f || shape is PlaneShape)
            {
                mass = 0f;
                inverseMass = 0f;
                inertiaBody = Mat3.Zero;
                inverseInertiaBody = Mat3.Zero;
            }
            else
            {
                mass = newMass;
                inverseMass = 1f / newMass;
                inertiaBody = shape.ComputeInertia(newMass);
                inverseInertiaBody = inertiaBody.Inverse;
            }
            UpdateDerived();
        }

        /// <summary>Mass from a density (kg/m^3) and the shape volume.</summary>
        public void SetDensity(float density)
        {
            SetMass(density * shape.Volume);
        }

        /// <summary>Overrides the body-space inertia tensor (e.g. computed from point masses).</summary>
        public void SetInertia(Mat3 bodyInertia)
        {
            inertiaBody = bodyInertia;
            inverseInertiaBody = bodyInertia.Inverse;
            UpdateDerived();
        }

        /// <summary>Recomputes R, I^-1(t) = R I_body^-1 R^T. Must be called after the orientation changes.</summary>
        public void UpdateDerived()
        {
            rotationMatrix = orientation.ToMat3();
            inverseInertiaWorld = Mat3.Similarity(rotationMatrix, inverseInertiaBody);
        }

        public Mat3 RotationMatrix => rotationMatrix;
        public Mat3 InverseInertiaWorld => inverseInertiaWorld;
        public Mat3 InertiaWorld => Mat3.Similarity(rotationMatrix, inertiaBody);

        /// <summary>v(t) = P(t) / M.</summary>
        public Vector3 Velocity
        {
            get => inverseMass > 0f ? linearMomentum * inverseMass : (isKinematic ? kinematicVelocity : VMath.Zero);
            set
            {
                if (inverseMass > 0f) linearMomentum = value * mass;
                else kinematicVelocity = value;
            }
        }

        /// <summary>w(t) = I^-1(t) L(t).</summary>
        public Vector3 AngularVelocity
        {
            get => inverseMass > 0f ? inverseInertiaWorld * angularMomentum : (isKinematic ? kinematicAngularVelocity : VMath.Zero);
            set
            {
                if (inverseMass > 0f) angularMomentum = InertiaWorld * value;
                else kinematicAngularVelocity = value;
            }
        }

        /// <summary>Velocity of a world point attached to the body: dr/dt = v + w x (r - x) (slide 35).</summary>
        public Vector3 GetPointVelocity(Vector3 worldPoint)
        {
            return Velocity + VMath.Cross(AngularVelocity, worldPoint - position);
        }

        public Vector3 LocalToWorld(Vector3 localPoint) => position + orientation.Rotate(localPoint);
        public Vector3 WorldToLocal(Vector3 worldPoint) => orientation.InverseRotate(worldPoint - position);
        public Vector3 LocalToWorldDirection(Vector3 localDir) => orientation.Rotate(localDir);

        // ---- Forces (chapter 2 part 2, slides 19-24 and 37) ----

        /// <summary>Force through the center of mass: translation only.</summary>
        public void AddForce(Vector3 f)
        {
            force += f;
            Wake();
        }

        /// <summary>Force applied at a world point: F adds to dP/dt, and tau = (r - x) x F adds to dL/dt.</summary>
        public void AddForceAtPoint(Vector3 f, Vector3 worldPoint)
        {
            force += f;
            torque += VMath.Cross(worldPoint - position, f);
            Wake();
        }

        public void AddTorque(Vector3 t)
        {
            torque += t;
            Wake();
        }

        /// <summary>Instantaneous change of momentum (impulse J = integral of F dt) through the center of mass.</summary>
        public void ApplyImpulse(Vector3 j)
        {
            if (inverseMass == 0f) return;
            linearMomentum += j;
        }

        /// <summary>Impulse at a world point: P += J, L += (r - x) x J.</summary>
        public void ApplyImpulseAtPoint(Vector3 j, Vector3 worldPoint)
        {
            if (inverseMass == 0f) return;
            linearMomentum += j;
            angularMomentum += VMath.Cross(worldPoint - position, j);
        }

        public void ApplyAngularImpulse(Vector3 j)
        {
            if (inverseMass == 0f) return;
            angularMomentum += j;
        }

        public void ClearAccumulators()
        {
            force = VMath.Zero;
            torque = VMath.Zero;
        }

        public void Wake()
        {
            isSleeping = false;
            sleepTimer = 0f;
        }

        public void Sleep()
        {
            isSleeping = true;
            linearMomentum = VMath.Zero;
            angularMomentum = VMath.Zero;
        }

        /// <summary>Kinetic energy 1/2 M v^2 + 1/2 w . L.</summary>
        public float KineticEnergy
        {
            get
            {
                if (inverseMass == 0f) return 0f;
                Vector3 v = Velocity;
                return 0.5f * mass * VMath.Dot(v, v) + 0.5f * VMath.Dot(AngularVelocity, angularMomentum);
            }
        }

        /// <summary>
        /// Time derivative of the state (slide 41): dx/dt = v, dq/dt = 1/2 w q, dP/dt = F, dL/dt = tau.
        /// </summary>
        public void StateDerivative(Vector3 totalForce, Vector3 totalTorque,
                                    out Vector3 dx, out Quat dq, out Vector3 dP, out Vector3 dL)
        {
            dx = Velocity;
            dq = Quat.Derivative(orientation, AngularVelocity);
            dP = totalForce;
            dL = totalTorque;
        }
    }
}
