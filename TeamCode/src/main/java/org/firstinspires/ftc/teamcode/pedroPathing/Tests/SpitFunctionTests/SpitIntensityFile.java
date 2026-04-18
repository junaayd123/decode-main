package org.firstinspires.ftc.teamcode.pedroPathing.Tests.SpitFunctionTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_Intensity;

@TeleOp(name = "Bot C Auto-Spit Only", group = "A_TeleOp")
public class SpitIntensityFile extends OpMode {

    private ColorSensors_Intensity intensitySensors;
    private DcMotor intake;

    private boolean intakeRunning   = false;
    private boolean outtakeActive   = false;
    private double  outtakeStartSec = -1.0;

    // Latch for RB
    private boolean rbWasReleased = true;

    // Sensor states
    private boolean intRightRaw;
    private boolean intBackRaw;
    private boolean intLeftRaw;

    private static final double INTAKE_POWER         = -1.0;
    private static final double OUTTAKE_POWER        =  1.0;
    private static final double OUTTAKE_DURATION_SEC =  0.25;

    @Override
    public void init() {
        intensitySensors = new ColorSensors_Intensity(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Ready: RB to start Intake. Auto-spit when full.");
    }

    @Override
    public void start() {
        intensitySensors.calibrateAmbientFloor();
        intakeRunning = false;
        outtakeActive = false;
    }

    @Override
    public void loop() {
        // 1. Update Sensors
        updateBallStates();

        // 2. Controller Input: ONE JOB
        // If RB is pressed, just turn on the intake.
        if (gamepad1.right_bumper && rbWasReleased) {
            intakeRunning = true;
            rbWasReleased = false;
        } else if (!gamepad1.right_bumper) {
            rbWasReleased = true;
        }

        // 3. The "Spit Function": Only runs if intake is actually on
        if (intakeRunning && !outtakeActive && isFull()) {
            // Once full, we stop the intake and start the spit timer
            intakeRunning = false;
            startOuttake(getRuntime());
        }

        // 4. Apply Motor Power
        applyMotorPower(getRuntime());

        // Telemetry
        telemetry.addData("Intake Running", intakeRunning);
        telemetry.addData("Spitting", outtakeActive);
        telemetry.addData("Sensors R|B|L", "%s | %s | %s", intRightRaw, intBackRaw, intLeftRaw);
        telemetry.update();
    }

    private void applyMotorPower(double nowSec) {
        if (outtakeActive) {
            // Check if spit duration is finished
            if (nowSec - outtakeStartSec <= OUTTAKE_DURATION_SEC) {
                intake.setPower(OUTTAKE_POWER);
            } else {
                outtakeActive = false;
                intake.setPower(0);
            }
        } else {
            // Normal intake operation
            intake.setPower(intakeRunning ? INTAKE_POWER : 0);
        }
    }

    private void startOuttake(double nowSec) {
        outtakeActive = true;
        outtakeStartSec = nowSec;
    }

    private void updateBallStates() {
        intensitySensors.update();
        intRightRaw = intensitySensors.rightHasBallRaw();
        intBackRaw  = intensitySensors.backHasBallRaw();
        intLeftRaw  = intensitySensors.leftHasBallRaw();
    }

    private boolean isFull() {
        return intRightRaw && intBackRaw && intLeftRaw;
    }
}