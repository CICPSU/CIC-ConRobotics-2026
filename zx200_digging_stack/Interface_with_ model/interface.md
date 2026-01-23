# Unified Joint Command Interface (Simulation ↔ Physical Model)

This document defines the **fixed command and feedback interface** used across
simulation (Isaac Sim) and the physical excavator scale model.

Once this interface is respected, **the same command stream can drive both
simulation and hardware without modification**.

This interface is treated as a **contract**.

---

## 1. Purpose

The goal of this interface is to guarantee:

- Identical command semantics for simulation and physical models
- Clear separation between high-level motion generation and low-level actuation
- Long-term maintainability when switching between:
  - Isaac Sim
  - Physical scale models
  - Future real machines

No downstream system is allowed to reinterpret this interface.

---

## 2. Command Interface (PC → Robot)

### Topic

/joint_command

### Message Type

sensor_msgs/JointState

### Semantics

| Field | Description |
|------|-------------|
| name | Canonical joint names (e.g., boom_joint) |
| position | Target joint positions **in radians** |

### Rules

- Joint names **must use canonical naming**
- Units are **always radians**
- velocity and effort fields are unused
- Message order must match between name[] and position[]

### Example

name: [swing_joint, boom_joint, arm_joint, bucket_joint]
position: [0.0, -0.45, 0.85, -0.30]

---

## 3. State Feedback Interface (Robot → PC)

### Topic

/joint_states

### Message Type

sensor_msgs/JointState

### Semantics

| Field | Description |
|------|-------------|
| name | Same canonical joint names as /joint_command |
| position | Estimated joint positions **in radians** |

---

## 4. Canonical Joint Names

- swing_joint
- boom_joint
- arm_joint
- bucket_joint
- bucket_end_joint

---

## 5. Unit Convention (Critical)

All joint values **must be radians** across:

- Task specification
- IK / trajectory generation
- ROS 2 interface

Any unit conversion must be handled **inside the robot backend only**.

---

## 6. Backend Responsibility

TBD

---

## 7. Guarantee

If this interface is respected, the same command stream can drive:

- Isaac Sim
- Raspberry Pi–based scale models
- Future physical machines
