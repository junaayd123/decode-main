# FTC Team 23918 — DECODE (2025–2026) Robot Code

This repository contains the full control system and autonomous software stack for FTC Team 23918 for the **DECODE (2025–2026)** competition season. It is built on top of the official FIRST Tech Challenge SDK and includes our custom implementations for autonomous navigation, vision, and control.

## Overview

Our software focuses on:

- Reliable autonomous scoring
- Robust localization and relocalization
- Path following with spline / Bézier trajectories
- Closed-loop control using PID
- Vision integration (AprilTags, object detection, and sensor fusion)

While multiple team members contribute to this codebase, the overall architecture, autonomous framework, and control systems are maintained by the software team.

## Repository Structure

```
├── FtcRobotController/    # Official FTC SDK controller module
└── TeamCode/              # All team-written code
    ├── Autonomous routines
    ├── Drivebase + subsystem abstractions
    ├── Localization and odometry
    ├── Vision pipelines
    └── Motion control and trajectory following
```

## Key Features

### Autonomous Navigation

- Bézier / spline path generation
- Pure pursuit–style and PID-based path following
- AprilTag-based relocalization
- GoBilda Pinpoint + odometry integration

### Control Systems

- PID control for slides, arm, and drivetrain
- Motion profiling for consistent scoring

### Vision

- AprilTag detection for alignment and pose correction
- Color / object detection pipelines (OpenCV + FTC VisionPortal)

### Reliability

- Recovery routines for failed intakes, misalignment, or partial cycles
- State-machine based autonomous structure
- Extensive telemetry and logging

## Requirements

- **Android Studio Ladybug (2024.2)** or later
- **Java 8** (as required by FTC SDK)
- REV Control Hub / Driver Hub or equivalent FTC-legal hardware

## Setup Instructions

1. Clone this repository:

   ```bash
   git clone https://github.com/junaayd123/decode-main.git
   ```

2. Open the project in **Android Studio**

3. Let Gradle sync and install dependencies

4. Connect to the robot and deploy using the standard FTC workflow

> [!NOTE]
> If you are new to FTC programming, refer to the [official FTC documentation](https://ftc-docs.firstinspires.org).

## Development Philosophy

Our goal is not just to make the robot work, but to make it:

- **Predictable**
- **Debuggable**
- **Modular**
- **Competition-resilient**

Most systems are written with reusability, testability, and failure recovery in mind.

## Credits

This codebase is developed by **FTC Team 23918**. Primary contributors change season to season, but the overall architecture and autonomous framework are maintained by the software lead team.

Built on top of the [official FIRST Tech Challenge SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController).

## License

This project follows the same license and usage rules as the FTC SDK.

---

**Good luck to all FTC teams this season! 🤖**
