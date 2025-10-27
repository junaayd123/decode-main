package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ChamberRotator", group = "Tutorial")
public class InfiniteMotor_Test extends LinearOpMode {

    private CRServo sorter;
    private DcMotorEx encoder; // through-bore encoder

    private int chamberCount = 3;
    private double ticksPerRevolution = 8192; // REV Through Bore encoder
    private int currentChamber = 0;
    private boolean buttonPressed = false;

    @Override
    public void runOpMode() {
        sorter = hardwareMap.get(CRServo.class, "sorter");
        encoder = hardwareMap.get(DcMotorEx.class, "encoder"); // name it in config

        // Reset encoder position
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready to spin chambers!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double position = encoder.getCurrentPosition();
            double degrees = (position / ticksPerRevolution) * 360;

            // If square pressed, move to next chamber
            if (gamepad1.square && !buttonPressed) {
                buttonPressed = true;
                currentChamber = (currentChamber + 1) % chamberCount;
                double targetDegrees = currentChamber * (360.0 / chamberCount);

                // Spin until target is reached
                double power = 0.3; // adjust for speed
                sorter.setPower(power);

                if (targetDegrees < degrees) targetDegrees += 360; // wraparound handling

                while (opModeIsActive() && ((degrees % 360) < targetDegrees - 5)) {
                    degrees = (encoder.getCurrentPosition() / ticksPerRevolution) * 360;
                    telemetry.addData("Current", degrees % 360);
                    telemetry.addData("Target", targetDegrees % 360);
                    telemetry.update();
                }

                sorter.setPower(0); // stop at next chamber
            }

            // Reset flag when button released
            if (!gamepad1.square) {
                buttonPressed = false;
            }

            telemetry.addData("Chamber", currentChamber);
            telemetry.addData("Encoder", position);
            telemetry.update();
        }
    }
}
