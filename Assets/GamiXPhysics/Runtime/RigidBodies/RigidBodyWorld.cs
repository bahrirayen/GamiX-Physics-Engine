using System.Collections.Generic;
using GamiX.Physics.Collision;
using GamiX.Physics.Geometry;
using UnityEngine;

namespace GamiX.Physics.RigidBodies
{
    public enum RigidIntegrator
    {
        /// <summary>Velocity first, then position with the new velocity. Recommended with contacts.</summary>
        SemiImplicitEuler,
        /// <summary>X(t + dt) = X(t) + dt dX/dt (chapter 2 part 2, slide 42).</summary>
        ExplicitEuler,
        /// <summary>RK4 on the full state (x, q, P, L), forces held constant over the step.</summary>
        RungeKutta4
    }

    /// <summary>A contact between two rigid bodies, with the impulses found by the solver.</summary>
    public sealed class RigidContact
    {
        public RigidBody a, b;
        public Vector3 point;
        /// <summary>Unit normal pointing from A to B.</summary>
        public Vector3 normal;
        public float penetration;

        /// <summary>Accumulated normal impulse of the last step (N.s). Useful for breaking/fracture logic.</summary>
        public float normalImpulse;
        public float tangentImpulse1, tangentImpulse2;

        internal Vector3 rA, rB, t1, t2;
        internal float massN, massT1, massT2;
        internal float velocityBias;
        internal float friction, restitution;
        internal bool aActive, bActive;
    }

    /// <summary>Something that adds forces to rigid bodies every step (springs, wind, motors...).</summary>
    public interface IRigidForceGenerator
    {
        void Apply(RigidBodyWorld world, float dt);
    }

    /// <summary>
    /// Damped spring between two anchor points attached to bodies (or one body and a fixed world point).
    /// The force is applied at the anchors, so it also creates torques (tau = r x F).
    /// When the force exceeds <see cref="breakForce"/>, the spring breaks.
    /// </summary>
    public sealed class RigidSpring : IRigidForceGenerator
    {
        public RigidBody bodyA, bodyB;
        public Vector3 localAnchorA, localAnchorB;
        /// <summary>World anchor used when bodyB is null.</summary>
        public Vector3 worldAnchor;
        public float restLength, stiffness, damping;
        public float breakForce = float.PositiveInfinity;
        public bool broken;
        public float lastForce;

        public Vector3 AnchorA => bodyA.LocalToWorld(localAnchorA);
        public Vector3 AnchorB => bodyB != null ? bodyB.LocalToWorld(localAnchorB) : worldAnchor;

        public void Apply(RigidBodyWorld world, float dt)
        {
            if (broken)
                return;
            Vector3 pa = AnchorA, pb = AnchorB;
            Vector3 d = pb - pa;
            float len = VMath.Length(d);
            if (len < VMath.Epsilon)
                return;
            Vector3 dir = d / len;
            Vector3 va = bodyA.GetPointVelocity(pa);
            Vector3 vb = bodyB != null ? bodyB.GetPointVelocity(pb) : VMath.Zero;
            float magnitude = stiffness * (len - restLength) + damping * VMath.Dot(vb - va, dir);
            lastForce = Mathf.Abs(magnitude);
            if (lastForce > breakForce)
            {
                broken = true;
                return;
            }
            Vector3 f = dir * magnitude;
            bodyA.AddForceAtPoint(f, pa);
            bodyB?.AddForceAtPoint(-f, pb);
        }
    }

    /// <summary>
    /// Rigid body simulation loop of chapter 4, slide 72:
    /// integration -> bounding volumes -> broad phase -> narrow phase -> contacts (p, n, delta) -> response.
    /// The response is an iterative impulse solver. Each contact impulse uses Baraff's effective mass
    /// j = -(1 + e) v_rel / (1/mA + 1/mB + n.((IA^-1 (rA x n)) x rA) + n.((IB^-1 (rB x n)) x rB)),
    /// with Coulomb friction clamped by mu * j.
    /// </summary>
    public sealed class RigidBodyWorld
    {
        public readonly List<RigidBody> Bodies = new List<RigidBody>();
        public readonly List<IRigidForceGenerator> ForceGenerators = new List<IRigidForceGenerator>();

        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        public RigidIntegrator integrator = RigidIntegrator.SemiImplicitEuler;
        public int substeps = 1;
        public int solverIterations = 12;

        /// <summary>Fraction of the penetration corrected per step (Baumgarte stabilization).</summary>
        public float baumgarte = 0.2f;
        /// <summary>Penetration allowed without correction, avoids jitter.</summary>
        public float penetrationSlop = 0.005f;
        /// <summary>Below this approach speed (m/s) contacts do not bounce.</summary>
        public float restitutionThreshold = 0.5f;
        public float aabbMargin = 0.02f;

        public bool allowSleeping = true;
        public float sleepLinearVelocity = 0.05f;
        public float sleepAngularVelocity = 0.08f;
        public float timeToSleep = 0.5f;

        public IBroadPhase broadPhase = new SweepAndPruneBroadPhase();

        /// <summary>Contacts of the last sub-step (read-only use).</summary>
        public readonly List<RigidContact> Contacts = new List<RigidContact>();

        public int PairCount { get; private set; }
        public float time;

        private readonly List<Aabb> aabbs = new List<Aabb>();
        private readonly List<BodyPair> pairs = new List<BodyPair>();
        private readonly List<Contact> narrowContacts = new List<Contact>();
        private readonly List<RigidContact> contactPool = new List<RigidContact>();
        private int nextId;

        public RigidBody AddBody(RigidBody body)
        {
            body.id = nextId++;
            Bodies.Add(body);
            return body;
        }

        public void RemoveBody(RigidBody body)
        {
            Bodies.Remove(body);
        }

        public void Step(float dt)
        {
            int n = substeps < 1 ? 1 : substeps;
            float h = dt / n;
            for (int s = 0; s < n; s++)
                SubStep(h);
        }

        private void SubStep(float h)
        {
            for (int i = 0; i < ForceGenerators.Count; i++)
                ForceGenerators[i].Apply(this, h);

            // 1) Integration.
            if (integrator == RigidIntegrator.SemiImplicitEuler)
                IntegrateVelocities(h);
            else
                IntegrateFullState(h);

            // 2-4) Bounding volumes, broad phase, narrow phase.
            DetectContacts();

            // 5) Response.
            PrepareContacts(h);
            for (int it = 0; it < solverIterations; it++)
                for (int c = 0; c < Contacts.Count; c++)
                    SolveContact(Contacts[c]);

            if (integrator == RigidIntegrator.SemiImplicitEuler)
                IntegratePositions(h);
            else
                ProjectPositions();

            UpdateSleep(h);
            for (int i = 0; i < Bodies.Count; i++)
                Bodies[i].ClearAccumulators();
            time += h;
        }

        // ---------------------------------------------------------------- integration

        private void IntegrateVelocities(float h)
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                if (!b.IsDynamic || b.isSleeping)
                    continue;
                Vector3 f = b.force;
                if (b.useGravity)
                    f += gravity * b.mass;
                // dP/dt = F, dL/dt = tau.
                b.linearMomentum += f * h;
                b.angularMomentum += b.torque * h;
                b.linearMomentum *= 1f / (1f + h * b.linearDamping);
                b.angularMomentum *= 1f / (1f + h * b.angularDamping);
            }
        }

        private void IntegratePositions(float h)
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                if (b.isSleeping || (!b.IsDynamic && !b.isKinematic))
                    continue;
                // dx/dt = v, dq/dt = 1/2 w q (integrated exactly for a constant w over the step).
                b.position += b.Velocity * h;
                b.orientation = Quat.IntegrateExact(b.orientation, b.AngularVelocity, h);
                b.UpdateDerived();
            }
        }

        private struct BodyState
        {
            public Vector3 x, p, l;
            public Quat q;
        }

        private static BodyState Derivative(RigidBody b, BodyState s, Vector3 f, Vector3 tau)
        {
            // Evaluate v and w for the intermediate state s (I^-1(t) = R I_body^-1 R^T with R from s.q).
            Quat q = s.q.Normalized;
            Mat3 r = q.ToMat3();
            Mat3 invI = Mat3.Similarity(r, b.inverseInertiaBody);
            Vector3 v = s.p * b.inverseMass;
            Vector3 w = invI * s.l;
            return new BodyState { x = v, q = Quat.Derivative(q, w), p = f, l = tau };
        }

        private static BodyState Add(BodyState s, BodyState d, float h)
        {
            return new BodyState { x = s.x + d.x * h, q = s.q + d.q * h, p = s.p + d.p * h, l = s.l + d.l * h };
        }

        /// <summary>
        /// Explicit Euler or RK4 on X(t) = (x, q, P, L), dX/dt = (v, 1/2 w q, F, tau) (slides 41-45).
        /// </summary>
        private void IntegrateFullState(float h)
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                if (b.isKinematic)
                {
                    b.position += b.kinematicVelocity * h;
                    b.orientation = Quat.IntegrateExact(b.orientation, b.kinematicAngularVelocity, h);
                    b.UpdateDerived();
                    continue;
                }
                if (!b.IsDynamic || b.isSleeping)
                    continue;
                Vector3 f = b.force + (b.useGravity ? gravity * b.mass : VMath.Zero);
                Vector3 tau = b.torque;
                BodyState s0 = new BodyState { x = b.position, q = b.orientation, p = b.linearMomentum, l = b.angularMomentum };
                BodyState next;
                if (integrator == RigidIntegrator.ExplicitEuler)
                {
                    next = Add(s0, Derivative(b, s0, f, tau), h);
                }
                else
                {
                    BodyState k1 = Derivative(b, s0, f, tau);
                    BodyState k2 = Derivative(b, Add(s0, k1, 0.5f * h), f, tau);
                    BodyState k3 = Derivative(b, Add(s0, k2, 0.5f * h), f, tau);
                    BodyState k4 = Derivative(b, Add(s0, k3, h), f, tau);
                    float h6 = h / 6f;
                    next = new BodyState
                    {
                        x = s0.x + (k1.x + 2f * k2.x + 2f * k3.x + k4.x) * h6,
                        q = s0.q + (k1.q + k2.q * 2f + k3.q * 2f + k4.q) * h6,
                        p = s0.p + (k1.p + 2f * k2.p + 2f * k3.p + k4.p) * h6,
                        l = s0.l + (k1.l + 2f * k2.l + 2f * k3.l + k4.l) * h6
                    };
                }
                b.position = next.x;
                b.orientation = next.q.Normalized;
                b.linearMomentum = next.p * (1f / (1f + h * b.linearDamping));
                b.angularMomentum = next.l * (1f / (1f + h * b.angularDamping));
                b.UpdateDerived();
            }
        }

        // ---------------------------------------------------------------- detection

        private void DetectContacts()
        {
            aabbs.Clear();
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                aabbs.Add(b.shape.ComputeAabb(b.position, b.orientation).Expanded(aabbMargin));
            }

            broadPhase.FindPairs(aabbs, pairs);
            PairCount = pairs.Count;

            Contacts.Clear();
            int used = 0;
            for (int p = 0; p < pairs.Count; p++)
            {
                RigidBody a = Bodies[pairs[p].a], b = Bodies[pairs[p].b];
                if (!a.IsDynamic && !b.IsDynamic)
                    continue;
                if ((a.layer & b.collisionMask) == 0 || (b.layer & a.collisionMask) == 0)
                    continue;
                bool aAwake = a.IsDynamic && !a.isSleeping;
                bool bAwake = b.IsDynamic && !b.isSleeping;
                if (!aAwake && !bAwake && !a.isKinematic && !b.isKinematic)
                    continue;

                narrowContacts.Clear();
                if (!Collide(a, b, narrowContacts))
                    continue;

                // A moving body wakes up the sleeping body it touches.
                if (aAwake && b.isSleeping && a.sleepTimer == 0f) b.Wake();
                if (bAwake && a.isSleeping && b.sleepTimer == 0f) a.Wake();
                if (a.isKinematic && b.isSleeping) b.Wake();
                if (b.isKinematic && a.isSleeping) a.Wake();

                for (int k = 0; k < narrowContacts.Count; k++)
                {
                    if (used == contactPool.Count)
                        contactPool.Add(new RigidContact());
                    RigidContact c = contactPool[used++];
                    Contact nc = narrowContacts[k];
                    c.a = a;
                    c.b = b;
                    c.point = nc.point;
                    c.normal = nc.normal;
                    c.penetration = nc.penetration;
                    Contacts.Add(c);
                }
            }
        }

        /// <summary>Dispatch on the shape pair. Contacts are returned with normals from A to B.</summary>
        public static bool Collide(RigidBody a, RigidBody b, List<Contact> contacts)
        {
            RigidShapeType ta = a.shape.Type, tb = b.shape.Type;
            int start = contacts.Count;
            bool flip = false;
            bool hit;

            if (ta == RigidShapeType.Sphere && tb == RigidShapeType.Sphere)
            {
                hit = NarrowPhase.SphereSphere(SphereOf(a), SphereOf(b), contacts);
            }
            else if (ta == RigidShapeType.Sphere && tb == RigidShapeType.Box)
            {
                hit = NarrowPhase.SphereObb(SphereOf(a), ObbOf(b), contacts);
            }
            else if (ta == RigidShapeType.Box && tb == RigidShapeType.Sphere)
            {
                hit = NarrowPhase.SphereObb(SphereOf(b), ObbOf(a), contacts);
                flip = true;
            }
            else if (ta == RigidShapeType.Box && tb == RigidShapeType.Box)
            {
                hit = NarrowPhase.ObbObb(ObbOf(a), ObbOf(b), contacts);
            }
            else if (ta == RigidShapeType.Sphere && tb == RigidShapeType.Plane)
            {
                hit = NarrowPhase.SpherePlane(SphereOf(a), PlaneOf(b), contacts);
            }
            else if (ta == RigidShapeType.Plane && tb == RigidShapeType.Sphere)
            {
                hit = NarrowPhase.SpherePlane(SphereOf(b), PlaneOf(a), contacts);
                flip = true;
            }
            else if (ta == RigidShapeType.Box && tb == RigidShapeType.Plane)
            {
                hit = NarrowPhase.ObbPlane(ObbOf(a), PlaneOf(b), contacts);
            }
            else if (ta == RigidShapeType.Plane && tb == RigidShapeType.Box)
            {
                hit = NarrowPhase.ObbPlane(ObbOf(b), PlaneOf(a), contacts);
                flip = true;
            }
            else
            {
                hit = false;
            }

            if (hit && flip)
            {
                for (int i = start; i < contacts.Count; i++)
                {
                    Contact c = contacts[i];
                    c.normal = -c.normal;
                    contacts[i] = c;
                }
            }
            return hit;
        }

        private static Sphere SphereOf(RigidBody b) => new Sphere(b.position, ((SphereShape)b.shape).radius);
        private static Obb ObbOf(RigidBody b) => new Obb(b.position, b.orientation, ((BoxShape)b.shape).halfExtents);
        private static Plane3D PlaneOf(RigidBody b) => ((PlaneShape)b.shape).ToPlane(b.position, b.orientation);

        // ---------------------------------------------------------------- response

        private static float InvMass(RigidBody b, bool active) => active ? b.inverseMass : 0f;

        private static Vector3 AngularTerm(RigidBody b, bool active, Vector3 r, Vector3 axis)
        {
            if (!active) return VMath.Zero;
            // (I^-1 (r x n)) x r
            return VMath.Cross(b.InverseInertiaWorld * VMath.Cross(r, axis), r);
        }

        private float EffectiveMass(RigidContact c, Vector3 axis)
        {
            float k = InvMass(c.a, c.aActive) + InvMass(c.b, c.bActive)
                      + VMath.Dot(AngularTerm(c.a, c.aActive, c.rA, axis) + AngularTerm(c.b, c.bActive, c.rB, axis), axis);
            return k > 1e-12f ? 1f / k : 0f;
        }

        private static Vector3 RelativeVelocity(RigidContact c)
        {
            Vector3 va = c.a.Velocity + VMath.Cross(c.a.AngularVelocity, c.rA);
            Vector3 vb = c.b.Velocity + VMath.Cross(c.b.AngularVelocity, c.rB);
            return vb - va;
        }

        private void PrepareContacts(float h)
        {
            bool positionBias = integrator == RigidIntegrator.SemiImplicitEuler;
            for (int i = 0; i < Contacts.Count; i++)
            {
                RigidContact c = Contacts[i];
                c.aActive = c.a.IsDynamic && !c.a.isSleeping;
                c.bActive = c.b.IsDynamic && !c.b.isSleeping;
                c.rA = c.point - c.a.position;
                c.rB = c.point - c.b.position;
                c.friction = Mathf.Sqrt(c.a.friction * c.b.friction);
                c.restitution = Mathf.Max(c.a.restitution, c.b.restitution);
                c.normalImpulse = c.tangentImpulse1 = c.tangentImpulse2 = 0f;

                Vector3 vrel = RelativeVelocity(c);
                float vn = VMath.Dot(vrel, c.normal);

                // Friction directions: first tangent along the sliding direction when there is one.
                Vector3 vt = vrel - c.normal * vn;
                float vtLen = VMath.Length(vt);
                if (vtLen > 1e-4f)
                {
                    c.t1 = vt / vtLen;
                    c.t2 = VMath.Cross(c.normal, c.t1);
                }
                else
                {
                    VMath.OrthonormalBasis(c.normal, out c.t1, out c.t2);
                }

                c.massN = EffectiveMass(c, c.normal);
                c.massT1 = EffectiveMass(c, c.t1);
                c.massT2 = EffectiveMass(c, c.t2);

                float bounce = vn < -restitutionThreshold ? -c.restitution * vn : 0f;
                float correction = positionBias ? baumgarte / h * Mathf.Max(c.penetration - penetrationSlop, 0f) : 0f;
                c.velocityBias = Mathf.Max(bounce, correction);
            }
        }

        private static void ApplyPairImpulse(RigidContact c, Vector3 impulse)
        {
            if (c.aActive) c.a.ApplyImpulseAtPoint(-impulse, c.point);
            if (c.bActive) c.b.ApplyImpulseAtPoint(impulse, c.point);
        }

        private static void SolveContact(RigidContact c)
        {
            // Friction (two tangents), bounded by the current normal impulse (Coulomb cone, box approximation).
            Vector3 vrel = RelativeVelocity(c);
            float maxFriction = c.friction * c.normalImpulse;

            float dt1 = -c.massT1 * VMath.Dot(vrel, c.t1);
            float old1 = c.tangentImpulse1;
            c.tangentImpulse1 = VMath.Clamp(old1 + dt1, -maxFriction, maxFriction);
            float dt2 = -c.massT2 * VMath.Dot(vrel, c.t2);
            float old2 = c.tangentImpulse2;
            c.tangentImpulse2 = VMath.Clamp(old2 + dt2, -maxFriction, maxFriction);
            ApplyPairImpulse(c, c.t1 * (c.tangentImpulse1 - old1) + c.t2 * (c.tangentImpulse2 - old2));

            // Normal impulse: drive the normal relative velocity to the bias (restitution / penetration recovery).
            vrel = RelativeVelocity(c);
            float vn = VMath.Dot(vrel, c.normal);
            float dj = c.massN * (c.velocityBias - vn);
            float oldN = c.normalImpulse;
            c.normalImpulse = Mathf.Max(oldN + dj, 0f); // Contacts can only push.
            ApplyPairImpulse(c, c.normal * (c.normalImpulse - oldN));
        }

        /// <summary>
        /// Separates interpenetrating bodies in proportion to their inverse mass (chapter 4, application 3),
        /// used with the Euler / RK4 integrators whose positions are already advanced.
        /// </summary>
        private void ProjectPositions()
        {
            for (int i = 0; i < Contacts.Count; i++)
            {
                RigidContact c = Contacts[i];
                float wa = InvMass(c.a, c.aActive), wb = InvMass(c.b, c.bActive);
                float w = wa + wb;
                float depth = c.penetration - penetrationSlop;
                if (w <= 0f || depth <= 0f)
                    continue;
                Vector3 corr = c.normal * (0.8f * depth / w);
                c.a.position -= corr * wa;
                c.b.position += corr * wb;
            }
        }

        private void UpdateSleep(float h)
        {
            if (!allowSleeping)
                return;
            float lin2 = sleepLinearVelocity * sleepLinearVelocity;
            float ang2 = sleepAngularVelocity * sleepAngularVelocity;
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                if (!b.IsDynamic || b.isSleeping || !b.canSleep)
                    continue;
                if (VMath.LengthSq(b.Velocity) < lin2 && VMath.LengthSq(b.AngularVelocity) < ang2)
                {
                    b.sleepTimer += h;
                    if (b.sleepTimer > timeToSleep)
                        b.Sleep();
                }
                else
                {
                    b.sleepTimer = 0f;
                }
            }
        }

        // ---------------------------------------------------------------- queries

        /// <summary>Closest body hit by the ray (chapter 3 ray casting applied to the scene).</summary>
        public bool Raycast(Ray3D ray, float maxDistance, out RigidBody body, out RayHit hit)
        {
            body = null;
            hit = default;
            float best = maxDistance;
            for (int i = 0; i < Bodies.Count; i++)
            {
                RigidBody b = Bodies[i];
                RayHit h;
                bool ok;
                switch (b.shape.Type)
                {
                    case RigidShapeType.Sphere: ok = Geometry.Raycast.RaySphere(ray, SphereOf(b), out h, best); break;
                    case RigidShapeType.Box: ok = Geometry.Raycast.RayObb(ray, ObbOf(b), out h, best); break;
                    default: ok = Geometry.Raycast.RayPlane(ray, PlaneOf(b), out h, best); break;
                }
                if (ok && h.t < best)
                {
                    best = h.t;
                    hit = h;
                    body = b;
                }
            }
            return body != null;
        }

        public Vector3 TotalLinearMomentum()
        {
            Vector3 p = VMath.Zero;
            for (int i = 0; i < Bodies.Count; i++)
                if (Bodies[i].IsDynamic)
                    p += Bodies[i].linearMomentum;
            return p;
        }

        public float TotalKineticEnergy()
        {
            float e = 0f;
            for (int i = 0; i < Bodies.Count; i++)
                e += Bodies[i].KineticEnergy;
            return e;
        }
    }
}
