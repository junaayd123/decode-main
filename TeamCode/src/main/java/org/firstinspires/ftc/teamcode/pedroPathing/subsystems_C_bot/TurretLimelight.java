package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretLimelight {

    public Servo turret;
    public Servo turret2;
//    public DcMotorEx TurretMotor;

    // Servo calibration points
    // Forward (0 ticks / 0 degrees) → 0.515
    // 180° right (+670 ticks)        → 0.08
    // 180° left  (-670 ticks)        → 0.95
    private static final double SERVO_FORWARD   = 0.515;
    private static final double SERVO_MAX_RIGHT = 0.08;
    private static final double SERVO_MAX_LEFT  = 0.95;

    // Tick/degree limits (kept for API compatibility)
    private static final double TURRET_MIN_TICKS = -670;
    private static final double TURRET_MAX_TICKS =  670;

    // Conversion: ticks → servo position
    // Right of center (positive ticks): FORWARD → MAX_RIGHT
    // Left  of center (negative ticks): FORWARD → MAX_LEFT
    private static final double TICKS_TO_SERVO_RIGHT = (SERVO_MAX_RIGHT - SERVO_FORWARD) / TURRET_MAX_TICKS; // negative slope
    private static final double TICKS_TO_SERVO_LEFT  = (SERVO_MAX_LEFT  - SERVO_FORWARD) / TURRET_MIN_TICKS; // negative slope

    // Degrees-to-ticks coefficient from old file (670 ticks = 180 deg)
    private static final double COEFFICIENT = 670.0 / 180.0;

    // State (kept public for dashboard/telemetry compatibility)
    public static int    target        = 0;
    public static double targetDegrees = 0.0;
    public static double tolerance     = 1.0;   // ticks, kept for API compat (unused by servos)
    public static double turetSpeed    = 0.8;   // unused by servos, kept for API compat
    public static double p = 0.005, i = 0.1, d = 0.0001; // unused, kept for API compat

    public double currentPos = 0.0; // last commanded tick position
    public double power      = 0.0; // unused, kept for API compat

    public Limelight3A limelight;
    boolean blueAlliance;
    double  groundDistanceCM;

    // -------------------------------------------------------------------------
    public TurretLimelight(HardwareMap hardwareMap) {
        turret  = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        // Start facing forward
        setServoPosition(0);
    }

    // -------------------------------------------------------------------------
    // Core: convert a tick value to a servo position and command both servos
    // -------------------------------------------------------------------------
    private void setServoPosition(double ticks) {
        // Clamp to physical limits
        ticks = Math.max(TURRET_MIN_TICKS, Math.min(TURRET_MAX_TICKS, ticks));
        currentPos = ticks;

        double servoPos;
        if (ticks >= 0) {
            // Moving right: linear interpolation between FORWARD and MAX_RIGHT
            servoPos = SERVO_FORWARD + ticks * TICKS_TO_SERVO_RIGHT;
        } else {
            // Moving left: linear interpolation between FORWARD and MAX_LEFT
            servoPos = SERVO_FORWARD + ticks * TICKS_TO_SERVO_LEFT;
        }

        // Clamp to [0, 1] as a safety net
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        turret.setPosition(servoPos);
        turret2.setPosition(servoPos);
    }

    // -------------------------------------------------------------------------
    // Public API — all original method signatures preserved
    // -------------------------------------------------------------------------

    /** No-op for servos; resets currentPos to 0 and moves to forward position. */
    public void resetTurretEncoder() {
        currentPos = 0.0;
        setServoPosition(0);
    }

    /** Updates currentPos from the last commanded position (servos have no encoder). */
    public void updateEncoderPos() {
        // Servos don't report position; currentPos reflects last command.
    }

    /** No-op for servos; kept for API compatibility. */
    public void setPid() { }

    /** Moves turret to the current static 'target' (in ticks). */
    public void toTargetInTicks() {
        setServoPosition(target);
    }

    /** Sets the tick target used by toTargetInTicks(). */
    public void setTicksTarget(int targett) {
        target = targett;
    }

    /** Moves turret to the current static 'targetDegrees'. */
    public void toTargetInDegrees() {
        double targetTicks = targetDegrees * COEFFICIENT;
        setServoPosition(targetTicks);
    }

    /** Moves turret to an explicit degree value (with clamping and wrapping). */
    public void toTargetInDegrees2(double targetDegrees2) {
        while (targetDegrees2 > 180) targetDegrees2 -= 360;
        while (targetDegrees2 < -180) targetDegrees2 += 360;

        double targetTicks = targetDegrees2 * COEFFICIENT;
        setServoPosition(targetTicks); // clamping handled inside setServoPosition
    }

    /** Sets the degree target used by toTargetInDegrees() (with wrapping). */
    public void setDegreesTarget(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        targetDegrees = deg;
    }

    public void setRedAlliance()  { blueAlliance = false; }
    public void setBlueAlliance() { blueAlliance = true;  }

    public double getTagDistance() { return groundDistanceCM; }
}