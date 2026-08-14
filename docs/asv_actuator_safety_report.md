# WAM-V MAVLink actuator safety gateway

## Status

The implementation, unit tests, and MAVROS 2.14 HIL overlay build are
verified. Native VRX latency results are deliberately **not** reported yet:
they must be gathered on Ubuntu 24.04 x86_64, not Apple Silicon amd64
emulation.

## 1. Modification

This addition introduces a VRX-specific safety boundary between an external
MAVLink producer and existing WAM-V Gazebo Thruster systems.

```text
External controller
  ├─ MAVLink HEARTBEAT
  └─ MAVLink HIL_ACTUATOR_CONTROLS (#93)
                 │ UDP / MAVLink v2
                 ▼
MAVROS + official MAVROS Extras HIL plugin
                 │ /mavros/state, /mavros/hil/actuator_controls
                 ▼
asv_actuator_safety_gateway
  ├─ command-freshness watchdog (0.2 s)
  ├─ connection / arming gate
  ├─ channel 0/1 → left/right WAM-V force mapping
  ├─ latched zero-command fail-safe and explicit reset
  └─ diagnostics and test instrumentation
                 ▼
Existing VRX Gazebo Thruster systems
```

MAVROS owns UDP, MAVLink framing, CRC, and connection state. The gateway adds
the missing VRX policy: an actuator stream can fail while heartbeat still
arrives, so stale commands are an independent fault and both outputs become
`0 N`.

`third_party/mavros_extras` is official MAVROS 2.14.0 source. Its build is
narrowed to upstream `hil.cpp`, matching Jazzy's MAVROS 2.14 binary core and
avoiding an in-house MAVLink parser.

## 2. Environment and verification method

Target environment: Ubuntu 24.04 x86_64, ROS 2 Jazzy, Gazebo Harmonic, VRX
`jazzy`, native Docker, and GCC.

```bash
docker build -f docker/Dockerfile.asv_actuator_safety -t vrx-asv-safety .
docker run --rm -it --network host vrx-asv-safety bash
```

Inside the container:

```bash
ros2 launch vrx_gz competition.launch.py headless:=true
ros2 run mavros mavros_node --ros-args \
  --params-file /ws/install/asv_actuator_safety_gateway/share/asv_actuator_safety_gateway/config/mavros_hil_plugins.yaml \
  -p fcu_url:=udp://:14560@
ros2 run asv_actuator_safety_gateway safety_gateway
./asv_safety_scripts/run_repeated_gateway_latency_test.sh --runs 50
```

The producer sends 1 Hz `HEARTBEAT` plus 50 Hz
`HIL_ACTUATOR_CONTROLS`; during fault injection it freezes only actuator
commands. The harness resets each latched fault, repeats the unarmed → zero →
armed recovery handshake, and records raw left/right native Gazebo Transport
setpoints.

`/wamv/thrusters/*/thrust/ang_vel` is the plugin's actuator angular-velocity
setpoint, not propeller sensor feedback. The primary measurement uses the local
monotonic clock: `T_zero_wall = first zero setpoint - last accepted callback`.
The dual-thruster stale impulse is integrated from actual monitor timestamps.
Stock results must be reported as right-censored `T_zero > 20 s`, never
infinity.

## 3. Verification results

| Check | Result | Evidence |
| --- | --- | --- |
| Gateway safety-policy tests | Pass, 4/4 | mapping; stale command with live connection; latch/reset handshake; connection loss |
| MAVROS 2.14 + official HIL overlay build | Pass | `mavros_extras` and gateway package build completed |
| Native x86_64 headless VRX startup | Pending | requires target host |
| Stock 20-second right-censored baseline | Pending | requires target host |
| Gateway repeated-run P95/max `T_zero_wall` | Pending | requires target host |
| Dual-thruster `I_stale` | Pending | requires target host |

The Apple Silicon amd64 build was a compatibility check only. Its QEMU timing
is deliberately excluded from safety-latency claims.
