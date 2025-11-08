package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "ChamberTuner", group = "Tuning")
public class ChamberTuner extends LinearOpMode {

    private DcMotor tbenc;
    private CRServo sorterServo;

    // Encoder constants
    private final int ANGLE_PER_ROTATION_ENCODER = 8192; // through-bore encoder counts per revolution
    private final double SERVO_POWER = 0.1; // adjust speed for fine control

    // Store recorded positions for each chamber
    private final Map<Character, Integer> chamberAngles = new HashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");
        sorterServo = hardwareMap.get(CRServo.class, "sorterservo");

        tbenc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbenc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== Chamber Tuner ===");
        telemetry.addLine("Use bumpers or joystick to rotate sorter");
        telemetry.addLine("Press A = mark Chamber A");
        telemetry.addLine("Press B = mark Chamber B");
        telemetry.addLine("Press X = mark Chamber C");
        telemetry.addLine("Press Y = print all angles");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual control of sorter using bumpers or left stick
            double manualPower = 0.0;

            if (gamepad1.left_bumper) {
                manualPower = -SERVO_POWER;
            } else if (gamepad1.right_bumper) {
                manualPower = SERVO_POWER;
            } else if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                manualPower = gamepad1.left_stick_x * SERVO_POWER;
            }

            sorterServo.setPower(manualPower);

            // Calculate current angle
            int deg = (int) Math.floor((double) -tbenc.getCurrentPosition() / (ANGLE_PER_ROTATION_ENCODER / 360.0));
            int currentAngle = Math.floorMod(deg, 360);

            // Button press to record positions
            if (gamepad1.a) {
                chamberAngles.put('A', currentAngle);
            }
            if (gamepad1.b) {
                chamberAngles.put('B', currentAngle);
            }
            if (gamepad1.x) {
                chamberAngles.put('C', currentAngle);
            }

            // Print all recorded positions when Y pressed
            if (gamepad1.y) {
                telemetry.addLine("=== Recorded Angles ===");
                for (Map.Entry<Character, Integer> entry : chamberAngles.entrySet()) {
                    telemetry.addData(String.valueOf(entry.getKey()), entry.getValue());
                }
                telemetry.addLine("Copy these into your Google Sheet!");
            }

            // Live telemetry
            telemetry.addLine("--- Live Data ---");
            telemetry.addData("Encoder Position (raw)", tbenc.getCurrentPosition());
            telemetry.addData("Angle (deg)", currentAngle);
            telemetry.addData("Manual Power", manualPower);
            telemetry.addLine("--- Recorded ---");
            telemetry.addData("A", chamberAngles.getOrDefault('A', null));
            telemetry.addData("B", chamberAngles.getOrDefault('B', null));
            telemetry.addData("C", chamberAngles.getOrDefault('C', null));
            telemetry.update();

            idle();
        }

        sorterServo.setPower(0);
    }
}
