package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Deposition_C {

    // --- Motors ---
    public DcMotorEx left;    // Has encoder
    public DcMotor right;     // Follower (no encoder)

    // --- PID controller for velocity ---
    private PIDController pid;

    // ---- PID constants (dashboard-tunable) ----
    public static double p = -0.001;
    public static double i = 0.0;
    public static double d = 0.0;
    public double farVelo_New = 1650;
    public double farVeloredauto = 1620;
    public double farVeloredauto2 = 1585;
    public double farVeloblue = 1600;
    //
    public double farVeloblueauto = 1620;
    public double farVeloblueauto2 = 1600;

    // Target velocity in ticks per second
    public  double targetVelocity = 0;

    // Optional: motor-specific constant for simple feedforward
    public static double kF = -0.00043;

    // --- Pre-set powers ---
    public double closePower = 0.56;
    public double farPower   = 0.70;
    //    public double farPower2  = 0.70;
    public double farVelo_auto = 1675;
    public double closeVelo_New = 1250; //use for new bot
    //public double closeVelo_New_auto = 1305; //use for new bot
    public double closeVelo_New_auto = 1280; //use for new bot


    private double powerOutput = 0.0;

    public Deposition_C(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotorEx.class, "depo");
        right = hardwareMap.get(DcMotor.class, "depo1");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(p, i, d);
    }
    public void setTargetVelocity(double target) {
        this.targetVelocity = target;
    }
    public boolean reachedTarget(){
        return targetVelocity != 0 && getVelocity() >= targetVelocity - 20 && getVelocity() <= targetVelocity + 20;
    }
    public boolean reachedTargetHighTolerance(){
        return targetVelocity != 0 && getVelocity() >= targetVelocity-40;
    }
    //

    // --- PID velocity update (call periodically in your loop) ---
    public void updatePID() {
        pid.setPID(p, i, d);

        double currentVelocity = left.getVelocity();  // ticks per second
        double pidOutput = pid.calculate(currentVelocity, targetVelocity);
        double ff = kF * targetVelocity;

        powerOutput = pidOutput + ff;
        powerOutput = Math.max(-1.0, Math.min(1.0, powerOutput));

        left.setPower(powerOutput);
        right.setPower(powerOutput);
    }

    // --- Manual control (driver mode) ---
    public void setPowerBoth(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    // --- Preset shooting powers ---
    public void shootClose() { setPowerBoth(closePower); }
    public void shootFar()   { setPowerBoth(farPower); }
//    public void shootFar2()  { setPowerBoth(farPower2); }

    // --- Stop both motors ---
    public void stop() {
        setPowerBoth(0.0);
    }

    // --- Getters for telemetry/debugging ---
    public double getVelocity() { return left.getVelocity(); }
    public double getTargetVelocity() { return targetVelocity; }
    public double getPowerOutput() { return powerOutput; }
}
//hi