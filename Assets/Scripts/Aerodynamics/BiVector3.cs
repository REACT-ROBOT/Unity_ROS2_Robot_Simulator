// Copyright 2024 Unity ROS2 Robot Simulator
// Licensed under the Apache License, Version 2.0
//
// BiVector3: Force and Torque combined structure
// Based on Aircraft-Physics by JoeBrow (MIT License)

using UnityEngine;

namespace Aerodynamics
{
    /// <summary>
    /// Represents a combined force and torque vector pair.
    /// Used for aggregating aerodynamic/hydrodynamic forces from multiple surfaces.
    /// </summary>
    public struct BiVector3
    {
        /// <summary>
        /// Force vector in Newtons (N)
        /// </summary>
        public Vector3 force;

        /// <summary>
        /// Torque vector in Newton-meters (N·m)
        /// </summary>
        public Vector3 torque;

        public BiVector3(Vector3 force, Vector3 torque)
        {
            this.force = force;
            this.torque = torque;
        }

        public static BiVector3 zero => new BiVector3(Vector3.zero, Vector3.zero);

        public static BiVector3 operator +(BiVector3 a, BiVector3 b)
        {
            return new BiVector3(a.force + b.force, a.torque + b.torque);
        }

        public static BiVector3 operator -(BiVector3 a, BiVector3 b)
        {
            return new BiVector3(a.force - b.force, a.torque - b.torque);
        }

        public static BiVector3 operator *(BiVector3 a, float scalar)
        {
            return new BiVector3(a.force * scalar, a.torque * scalar);
        }

        public static BiVector3 operator *(float scalar, BiVector3 a)
        {
            return new BiVector3(a.force * scalar, a.torque * scalar);
        }

        public override string ToString()
        {
            return $"Force: {force}, Torque: {torque}";
        }
    }
}
