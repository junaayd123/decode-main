package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Motor Directions (Pedro 2.0)", group = "Teleop Test")
public class MotorDirections extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private MultipleTelemetry telemetryA;

    @Override
    public void init() {
        // --- Initialize hardware ---
        leftFront = hardwareMap.get(DcMotorEx.class, "lfmotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfmotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "rbmotor");

        // Use Pedro 2.0 Constants for directions
        leftFront.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        leftRear.setDirection(Constants.driveConstants.leftRearMotorDirection);
        rightFront.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        rightRear.setDirection(Constants.driveConstants.rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        // Allow full RPM and float at zero power
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Pedro 2.0 Motor Direction Test");
        telemetryA.addLine("Press A → leftFront\nPress Y → leftRear\nPress B → rightFront\nPress X → rightRear");
        telemetryA.addLine("Change directions in Constants.Motors if any wheel spins the wrong way.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        // Refresh directions each loop in case Constants are modified live
        leftFront.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        leftRear.setDirection(Constants.driveConstants.leftRearMotorDirection);
        rightFront.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        rightRear.setDirection(Constants.driveConstants.rightRearMotorDirection);

        // Manual motor test
        leftFront.setPower(gamepad1.a ? 1.0 : 0.0);
        leftRear.setPower(gamepad1.y ? 1.0 : 0.0);
        rightFront.setPower(gamepad1.b ? 1.0 : 0.0);
        rightRear.setPower(gamepad1.x ? 1.0 : 0.0);

        telemetryA.addData("Left Front Direction", Constants.driveConstants.leftFrontMotorDirection);
        telemetryA.addData("Left Rear Direction", Constants.driveConstants.leftRearMotorDirection);
        telemetryA.addData("Right Front Direction", Constants.driveConstants.rightFrontMotorDirection);
        telemetryA.addData("Right Rear Direction", Constants.driveConstants.rightRearMotorDirection);
        telemetryA.addLine("Hold A/Y/B/X to spin respective motors at full power.");
        telemetryA.update();
    }
}
