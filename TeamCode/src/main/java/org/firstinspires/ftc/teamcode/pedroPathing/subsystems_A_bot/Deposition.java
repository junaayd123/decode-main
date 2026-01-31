package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Deposition {

    // --- Motors ---
    public DcMotorEx top;    // Has encoder
    public DcMotor bottom;     // Follower (no encoder)

    // --- PID controller for velocity ---
    private PIDController pid;

    // --- PID constants (dashboard-tunable) ---
    public static double p = 0.001;
    public static double i = 0.0;
    public static double d = 0.0;

    // Target velocity in ticks per second
    public  double targetVelocity = 0;

    // Optional: motor-specific constant for simple feedforward
    public static double kF = 0.00048;

    // --- Pre-set powers ---
    public double closePower = 0.56;
    public double farPower   = 0.70;
//    public double farPower2  = 0.70;
    public double closeVelo = -1300; //use for old bot
    public double farVelo = -1750; //use for old bot
    //
    public double closeVelo_New = 1125; //use for new bot
    public double closeVelo_New_auto = 1125; //use for new bot
    public double farVelo_New = 1650; //use for new bot
    public double farVelo_New_auto = 1480; //use for new bot
    // --- Internal variable for storing last output ---
    private double powerOutput = 0.0;

    public Deposition(HardwareMap hardwareMap) {
        top = hardwareMap.get(DcMotorEx.class, "depo1");
        bottom = hardwareMap.get(DcMotor.class, "depo");

        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(p, i, d);
    }
    public void setTargetVelocity(double target) {
        this.targetVelocity = target;
    }
    public boolean reachedTarget(){
        return targetVelocity != 0 && getVelocity() >= targetVelocity - 20 && getVelocity() <= targetVelocity + 20;
    }
    public boolean reachedTargetHighTolerance(){
        return targetVelocity != 0 && getVelocity() >= targetVelocity;
    }

    // --- PID velocity update (call periodically in your loop) ---
    public void updatePID() {
        pid.setPID(p, i, d);

        double currentVelocity = top.getVelocity();  // ticks per second
        double pidOutput = pid.calculate(currentVelocity, targetVelocity);
        double ff = kF * targetVelocity;

        powerOutput = pidOutput + ff;
        powerOutput = Math.max(-1.0, Math.min(1.0, powerOutput));

        top.setPower(powerOutput);
        bottom.setPower(powerOutput);
    }

    // --- Manual control (driver mode) ---
    public void setPowerBoth(double power) {
        top.setPower(power);
        bottom.setPower(power);
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
    public double getVelocity() { return top.getVelocity(); }
    public double getTargetVelocity() { return targetVelocity; }
    public double getPowerOutput() { return powerOutput; }
}
