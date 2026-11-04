using System;
using UnityEngine;

namespace GamiX.Physics.Unified
{
    /// <summary>Growable array of structs with by-reference access (no copies when updating an element).</summary>
    public sealed class StructList<T> where T : struct
    {
        public T[] items;
        public int Count;

        public StructList(int capacity = 16)
        {
            items = new T[Math.Max(1, capacity)];
        }

        public ref T this[int i] => ref items[i];

        public int Add(T item)
        {
            if (Count == items.Length)
                Array.Resize(ref items, items.Length * 2);
            items[Count] = item;
            return Count++;
        }

        public void Clear() => Count = 0;

        public void RemoveAtSwapBack(int i)
        {
            items[i] = items[Count - 1];
            Count--;
        }
    }

    /// <summary>
    /// Distance constraint C = |xi - xj| - L (stretch and bending of cloth and ropes, section 8).
    /// Optional tearing: the constraint is removed when stretched beyond tearRatio * L (water balloons, section 10).
    /// </summary>
    public struct DistanceConstraint
    {
        public int i, j;
        public float restLength;
        /// <summary>Stiffness in [0, 1] (1 = rigid link).</summary>
        public float stiffness;
        /// <summary>When &gt; 1, the constraint breaks once |xi - xj| &gt; tearRatio * restLength.</summary>
        public float tearRatio;
        public bool broken;
        /// <summary>Optional tag set by the builder (e.g. 0 = stretch, 1 = shear, 2 = bend).</summary>
        public int kind;

        // Per-iteration stiffness k' = 1 - (1 - k)^(1/iterations), cached for the iteration count it was computed for.
        internal float effectiveStiffness;
        internal int effectiveIterations;
        internal float effectiveSource;
    }

    /// <summary>
    /// Rigid body as a group of particles kept rigid by shape matching (section 5, eq. 15-16).
    /// Also stores the per-particle signed distance field (section 5.1).
    /// </summary>
    public sealed class RigidGroup
    {
        public int[] indices;
        /// <summary>r_i: rest offsets from the rest center of mass.</summary>
        public Vector3[] restOffsets;
        /// <summary>Signed distance phi_i (negative inside) of each particle in the rest shape.</summary>
        public float[] sdf;
        /// <summary>Unit SDF gradient of each particle, in rest (local) space.</summary>
        public Vector3[] sdfGradient;

        /// <summary>Shape matching stiffness in [0, 1]: lower values allow small elastic deformation (section 5.3).</summary>
        public float stiffness = 1f;
        /// <summary>Plastic deformation (section 5.3): deformation threshold and creep coefficient. 0 disables it.</summary>
        public float plasticThreshold = 0f;
        public float plasticCreep = 0f;

        /// <summary>Current pose found by shape matching: x_world = center + rotation * r_local.</summary>
        public Vector3 center;
        public Quat rotation = Quat.Identity;

        /// <summary>Rest center of mass in the local space the particles were sampled in (for mesh rendering).</summary>
        public Vector3 localRestCenter;
        public float totalMass;
        public int phase;
        public bool active = true;

        /// <summary>World position of a point given in the local (sampling) space of the shape.</summary>
        public Vector3 LocalToWorld(Vector3 localPoint) => center + rotation.Rotate(localPoint - localRestCenter);
    }

    /// <summary>A cloth piece: particle indices and triangles (rendering, aerodynamics, volume).</summary>
    public sealed class ClothHandle
    {
        public int[] particles;
        /// <summary>Indices into the solver particle arrays, 3 per triangle.</summary>
        public int[] triangles;
        public int width, height;
        public bool aerodynamics = true;
        public int phase;
    }
}
