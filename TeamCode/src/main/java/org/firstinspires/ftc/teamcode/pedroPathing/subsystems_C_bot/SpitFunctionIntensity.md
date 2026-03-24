# Intensity Intake/Outtake Reuse Guide

This guide explains how to copy the intake/outtake behavior from Bot C intensity teleop into another OpMode.

Source implementation:
- BotCTeleopBlueIntensity.java

## What this controller does

- Right bumper toggles intake on/off.
- Left bumper forces manual outtake while held.
- Auto outtake triggers when the chamber is detected as full.
- Full detection uses intensity sensors only (raw, sticky, and likely-full latch).
- A short confirm timer prevents one-frame false full triggers.

## Required hardware

- A DcMotor named intake in the hardware map.
- A ColorSensors_Intensity subsystem instance.

## Required state fields

Add these fields to your target class:

```java
private ColorSensors_Intensity intensitySensors;
private DcMotor intake;

private boolean prevRightBumper = false;
private boolean intakeRunning = false;
private boolean outtakeActive = false;
private double outtakeStartSec = -1.0;
private double fullCandidateStartSec = -1.0;

private boolean intRightRaw;
private boolean intBackRaw;
private boolean intLeftRaw;
private boolean intRightSticky;
private boolean intBackSticky;
private boolean intLeftSticky;
private boolean intLikelyFull;

private static final double INTAKE_POWER = -1.0;
private static final double OUTTAKE_POWER = 1.0;
private static final double OUTTAKE_DURATION_SEC = 0.40;
private static final double FULL_CONFIRM_SEC = 0.01;
```

## Init and start wiring

In init:

```java
intensitySensors = new ColorSensors_Intensity(hardwareMap);
intake = hardwareMap.get(DcMotor.class, "intake");
intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
prevRightBumper = false;
```

In start:

```java
resetRuntime();
intensitySensors.calibrateAmbientFloor();
intakeRunning = false;
outtakeActive = false;
outtakeStartSec = -1.0;
fullCandidateStartSec = -1.0;
intake.setPower(0);
```

## Loop pattern

Use this order each loop:

1. updateBallStates()
2. Handle controls (toggle and manual override)
3. runIntakeAndAutoOuttake(getRuntime())
4. Save edge-detection state

Example:

```java
updateBallStates();

boolean rbPressed = gamepad2.right_bumper && !prevRightBumper;

if (rbPressed) {
    if (intakeRunning || outtakeActive) {
        intakeRunning = false;
        stopOuttake();
        intake.setPower(0);
    } else {
        if (shouldOuttakeOnToggleOn()) {
            intakeRunning = false;
            startOuttake(getRuntime());
        } else {
            intakeRunning = true;
        }
    }
}

if (gamepad2.left_bumper) {
    intakeRunning = false;
    stopOuttake();
    intake.setPower(OUTTAKE_POWER);
} else {
    runIntakeAndAutoOuttake(getRuntime());
}

prevRightBumper = gamepad2.right_bumper;
```

## Core methods to copy

```java
private void runIntakeAndAutoOuttake(double nowSec) {
    if (outtakeActive) {
        if (nowSec - outtakeStartSec <= OUTTAKE_DURATION_SEC) {
            intake.setPower(OUTTAKE_POWER);
        } else {
            stopOuttake();
            fullCandidateStartSec = -1.0;
            intake.setPower(0);
        }
        return;
    }

    if (!intakeRunning) {
        fullCandidateStartSec = -1.0;
        intake.setPower(0);
        return;
    }

    boolean fullCandidate = isFullByGuardian() || isFullByIntensityCandidate();
    if (fullCandidate) {
        if (fullCandidateStartSec < 0) {
            fullCandidateStartSec = nowSec;
        }

        if (nowSec - fullCandidateStartSec >= FULL_CONFIRM_SEC) {
            intakeRunning = false;
            startOuttake(nowSec);
        }
    } else {
        fullCandidateStartSec = -1.0;
        intake.setPower(INTAKE_POWER);
    }
}

private void startOuttake(double nowSec) {
    outtakeActive = true;
    outtakeStartSec = nowSec;
    intake.setPower(OUTTAKE_POWER);
}

private void stopOuttake() {
    outtakeActive = false;
    outtakeStartSec = -1.0;
}

private boolean isFullByIntensityCandidate() {
    return isFullByIntensityRaw() || intLikelyFull;
}

private boolean isFullByGuardian() {
    int nowCount = 0;
    if (isRightNow()) nowCount++;
    if (isBackNow()) nowCount++;
    if (isLeftNow()) nowCount++;

    return intLikelyFull && nowCount >= 1;
}

private boolean shouldOuttakeOnToggleOn() {
    int nowCount = 0;
    if (isRightNow()) nowCount++;
    if (isBackNow()) nowCount++;
    if (isLeftNow()) nowCount++;

    return isFullByIntensityRaw()
            || (isFullByIntensitySticky() && nowCount >= 2)
            || (intLikelyFull && nowCount >= 2);
}

private boolean isRightNow() {
    return intRightRaw;
}

private boolean isBackNow() {
    return intBackRaw;
}

private boolean isLeftNow() {
    return intLeftRaw;
}

private boolean isFullByIntensityRaw() {
    return intRightRaw && intBackRaw && intLeftRaw;
}

private boolean isFullByIntensitySticky() {
    return intRightSticky && intBackSticky && intLeftSticky;
}

private void updateBallStates() {
    intensitySensors.update();

    intRightRaw = intensitySensors.rightHasBallRaw();
    intBackRaw = intensitySensors.backHasBallRaw();
    intLeftRaw = intensitySensors.leftHasBallRaw();

    intRightSticky = intensitySensors.rightHasBall();
    intBackSticky = intensitySensors.backHasBall();
    intLeftSticky = intensitySensors.leftHasBall();
    intLikelyFull = intensitySensors.isLikelyFull();
}
```

## Tuning tips

- OUTTAKE_DURATION_SEC: Increase if balls are not clearing.
- FULL_CONFIRM_SEC: Increase if auto-outtake false-triggers.
- INTAKE_POWER and OUTTAKE_POWER: Adjust for motor direction and mechanism.

## Common integration mistakes

- Not calling calibrateAmbientFloor() at start.
- Forgetting edge detection for right bumper toggle.
- Reversing intake/outtake signs for your motor direction.
- Calling runIntakeAndAutoOuttake before updateBallStates.
