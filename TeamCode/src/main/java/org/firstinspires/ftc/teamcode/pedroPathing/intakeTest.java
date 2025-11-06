package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "intakeTest", group = "TeleOp")
public class intakeTest extends LinearOpMode {

    private DcMotor intakeMotor;
    private CRServo sorter;

    @Override
    public void runOpMode() {
        // Hardware mapping
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sorter = hardwareMap.get(CRServo.class, "sorterservo");

        // Optional: reverse direction if needed
        // intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- PS4 + Logitech Button Mapping ---
            // PS4: X / O / Square / Triangle / PS
            // Logitech: A / B / X / Y / Guide

            boolean intakeOn     = gamepad1.cross || gamepad1.a;       // PS4 X / Logitech A
            boolean intakeOff    = gamepad1.circle || gamepad1.b;      // PS4 O / Logitech B
            boolean sorterOn     = gamepad1.square || gamepad1.x;      // PS4 Square / Logitech X
            boolean sorterOff    = gamepad1.triangle || gamepad1.y;    // PS4 Triangle / Logitech Y
            boolean runTogether  = gamepad1.ps || gamepad1.guide;      // PS4 PS / Logitech Guide

            if (runTogether) {
                // Run both together
                intakeMotor.setPower(1.0);
                sorter.setPower(0.25);
            }
            else if (intakeOn) {
                intakeMotor.setPower(1);
            }
            else if (intakeOff) {
                intakeMotor.setPower(0.0);
            }
            else if (sorterOn) {
                sorter.setPower(0.1);
            }
            else if (sorterOff) {
                sorter.setPower(0.0);
            }

            // Telemetry feedback
            telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
            telemetry.addData("Sorter Power", "%.2f", sorter.getPower());
            telemetry.update();
        }
    }
}
