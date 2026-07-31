# ROS 2 Integration Use Cases

[日本語](use-cases.ja.md)

This document explains when Topic, Service, and Action support are needed when integrating Hakoniwa with ROS 2 systems.

The goal of `hakoniwa-pdu-ros` is to preserve existing ROS 2 control assets while enabling Hakoniwa physics simulation, heterogeneous simulator integration, fault injection, and scenario execution.

## Interface Roles

| ROS 2 interface | Main purpose | Typical Hakoniwa integration |
| --- | --- | --- |
| Topic | Continuous data exchange | Sensors, state, control commands, actuator commands |
| Service | One-shot request and response | Initialization, reset, state queries, configuration, fault injection |
| Action | Long-running operations, progress, cancellation | Navigation, trajectory execution, missions, scenario execution |

## 1. Replace the Physics Simulator While Keeping Existing ROS Control

This use case keeps existing control nodes unchanged while replacing the physics and sensor generation currently provided by Gazebo or another simulator.

```text
ROS 2 control
    | commands
    v
Hakoniwa physics
    | sensor/state data
    v
ROS 2 control
```

Topic is the primary interface.

- ROS 2 sends velocity, attitude, joint, rotor, or other commands to Hakoniwa.
- Hakoniwa returns IMU, pose, velocity, joint state, and sensor data to ROS 2.

Service support is needed only when the existing control system calls simulator-specific operations such as initialization, reset, or environment configuration. Action is usually not required for the control loop itself.

## 2. Integrate Heterogeneous Simulators and Real Devices

This use case combines ROS 2 control, Hakoniwa drones, MuJoCo robots, environment simulators, and real ECUs in one verification environment.

Topic carries continuous data between components. Service becomes important because each component may require startup, initialization, mode switching, and status queries.

Action is useful when a component performs a long-running operation that needs progress reporting or cancellation.

## 3. Inject Faults and Change Environment Conditions

This is a major verification-oriented Hakoniwa use case.

Examples include:

- Rotor, actuator, and sensor fault injection
- Communication delay or loss configuration
- Wind, road, and obstacle changes
- Battery or vehicle parameter changes
- Object placement changes

These are commonly one-shot operations and therefore fit Service well.

```text
set_wind(...)
inject_rotor_failure(...)
reset_world()
set_sensor_noise(...)
```

Action is appropriate when an entire fault scenario is treated as one operation with progress, completion, and cancellation.

## 4. Operate Simulations and Verification Scenarios

Individual operations such as start, stop, reset, state query, and event injection can be exposed as Services.

```text
start()
stop()
reset()
get_state()
inject_event(...)
```

Action is a better fit when a multi-phase scenario is executed as a single unit.

```text
run_scenario()
  feedback: phase 2 / 5
  feedback: failure injected
  result: pass / fail
```

The more Hakoniwa is used as a verification execution platform rather than only as a physics engine, the more valuable Action support becomes.

## 5. Connect Nav2, MoveIt, and Mission Control

ROS 2 high-level robot frameworks commonly use Actions.

Examples include:

- `NavigateToPose`
- `FollowWaypoints`
- `FollowJointTrajectory`
- `ExecuteTrajectory`
- Long-running autonomous missions

Physics integration can still use Topics, but preserving existing ROS 2 applications and test environments may require Action support.

Action priority is relatively low for low-level drone control and basic physics replacement, but it becomes important for mobile robots, robot arms, and autonomous missions.

## Interface Demand by Use Case

| Use case | Topic | Service | Action |
| --- | --- | --- | --- |
| Physics simulator replacement | Required | Low to medium | Low |
| Sensor and control integration | Required | Low | Low |
| Initialization, reset, configuration | Medium | Required | Low |
| Fault injection and environment changes | Medium | High | Medium |
| Heterogeneous simulator integration | High | High | Medium |
| Scenario execution and test automation | Medium | High | High |
| Navigation integration | High | Medium | Required |
| Robot arm integration | High | Medium | Required |
| Long-running mission control | Medium | Medium | Required |

## Implementation Priority

### Topic

Topic support is required in almost every use case.

- Sensor data
- State data
- Control commands
- Actuator commands

### Service

Service support is important when Hakoniwa is used as an operable simulation and verification platform.

- Initialization and reset
- State queries
- Parameter changes
- Fault injection
- Environment changes
- Scenario event injection

With Service support, `hakoniwa-pdu-ros` becomes more than a data bridge: it becomes an integration layer for invoking Hakoniwa verification capabilities from ROS 2.

### Action

Action is not required by every user, but it is important for high-level robot frameworks and long-running tasks.

- Progress feedback
- Cancellation
- Long-running result handling
- Nav2, MoveIt, and mission-control integration
- Whole-scenario execution

## Development Direction

The recommended implementation order is:

1. Topic support for continuous control and state exchange
2. Service support for one-shot Hakoniwa operations
3. Action support for long-running tasks and high-level ROS 2 frameworks

Service support can reuse the existing `hakoniwa-pdu-rpc` and `hakoniwa-pdu-endpoint` implementations. Action should be introduced as a new capability while preserving compatibility with the existing Service implementation.

In summary:

```text
Topic   : Connect ROS 2 control and state loops to Hakoniwa
Service : Operate Hakoniwa physics, environment, fault, and scenario functions
Action  : Connect long-running tasks and high-level robot functions
```
