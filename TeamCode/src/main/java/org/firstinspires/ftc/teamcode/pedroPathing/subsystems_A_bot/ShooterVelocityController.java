package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Minimal PIDF velocity controller for two linked flywheel motors.
 * Tune all fields live in FTC Dashboard (because of @Config).
 *
 * Wire names assumed: "depo" (left) and "depo1" (right) in your TeleOp.
 * If you only have one motor, you can pass the same motor to both params.
 */
public class ShooterVelocityController {

    // ======= Tunables (Dashboard) =======
    public static double kP = 0.0009;
    public static double kI = 0.0002;
    public static double kD = 0.0000;

    // Base feedforward (roughly power needed to hold the target at steady-state)
    public static double kF_BASE = 0.16;

    // Optional battery compensation (multiplies kF by 12V/battV)
    public static boolean USE_BATTERY_FF = true;

    // Target RPM you can tweak live
    public static double TARGET_RPM = 2500.0;

    // “At speed” window
    public static double AT_SPEED_TOL_RPM = 75.0;

    // Integral clamp
    public static double I_MAX = 0.3;   // max accumulated I term (as power units)

    // RPM smoothing (EMA) for nicer telemetry
    public static boolean USE_EMA = true;
    public static double RPM_ALPHA = 0.35;

    // ======= Motor / encoder constants (SET THESE!) =======
    // TODO: Set this to your motor’s effective ticks per motor shaft revolution.
    //  - REV HD Hex 20:1 (bare) with built-in enc: 28 * gear ratio
    //  - GoBILDA Yellow Jacket with internal encoder: check vendor docs
    //  - External encoder on wheel: use that encoder’s counts per rev
    public static int TICKS_PER_REV = 28 * 20; // <--- EXAMPLE ONLY (560)

    // ======= Private state =======
    private final DcMotor left, right;
    private final VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetRpm = 0.0;

    private int lastTicksL, lastTicksR;
    private double lastTime = 0.0;
    private double rpmL, rpmR, rpmEma = 0.0;

    private double integral = 0.0;
    private double lastError = 0.0;

    public ShooterVelocityController(DcMotor left, DcMotor right, VoltageSensor battery) {
        this.left = left;
        this.right = right;
        this.battery = battery;

        // Recommended motor modes for velocity control using setPower()
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        right.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
// or FORWARD/FORWARD — depends on how yours are mounted. The key is: +power => outfeed for BOTH.


        // capture baselines
        lastTicksL = left.getCurrentPosition();
        lastTicksR = right.getCurrentPosition();
        timer.reset();
        lastTime = timer.seconds();
    }

    public void setTargetRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
    }

    public double getTargetRPM() { return targetRpm; }

    /** Call every loop. */
    public void update() {
        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 1e-3;

        int curTicksL = left.getCurrentPosition();
        int curTicksR = right.getCurrentPosition();

        double dTicksL = curTicksL - lastTicksL;
        double dTicksR = curTicksR - lastTicksR;

        // ticks/s -> rev/s -> RPM
        double rpsL = (dTicksL / dt) / TICKS_PER_REV;
        double rpsR = (dTicksR / dt) / TICKS_PER_REV;
        rpmL = rpsL * 60.0;
        rpmR = rpsR * 60.0;

        double rpm = 0.5 * (rpmL + rpmR);
        if (USE_EMA) {
            rpmEma = Double.isNaN(rpmEma) ? rpm : (RPM_ALPHA * rpm + (1.0 - RPM_ALPHA) * rpmEma);
        } else {
            rpmEma = rpm;
        }

        double error = targetRpm - rpmEma;

        // PID
        integral += error * dt * kI;
        // clamp integral (as power units)
        if (integral > I_MAX) integral = I_MAX;
        if (integral < -I_MAX) integral = -I_MAX;

        double derivative = (error - lastError) / dt;

        double pTerm = kP * error;
        double iTerm = integral;
        double dTerm = kD * derivative;

        // Feedforward (battery compensated if enabled)
        double ff = kF_BASE;
        if (USE_BATTERY_FF && battery != null) {
            double v = battery.getVoltage();
            if (v > 1e-3) ff = kF_BASE * (12.0 / v);
        }

        double power = ff + pTerm + iTerm + dTerm;

        // clip
        power = Math.max(-1.0, Math.min(1.0, power));

        left.setPower(power);
        right.setPower(power);

        // keep history
        lastTicksL = curTicksL;
        lastTicksR = curTicksR;
        lastTime = now;
        lastError = error;
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
        targetRpm = 0;
        integral = 0;
        lastError = 0;
    }

    public boolean isAtSpeed() {
        double rpm = rpmEma;
        return Math.abs(targetRpm - rpm) <= AT_SPEED_TOL_RPM && targetRpm > 0;
    }

    public double getRPM()    { return rpmEma; }
    public double getRPML()   { return rpmL; }
    public double getRPMR()   { return rpmR; }
}
