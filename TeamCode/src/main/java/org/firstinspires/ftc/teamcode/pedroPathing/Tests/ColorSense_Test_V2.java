package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_New;

@TeleOp(name = "ColorSense_Test_V2")
public class ColorSense_Test_V2 extends LinearOpMode {
    Servo led;
    DcMotor intake;
    ColorSensors_New colorSensors;

    String sensorPosition = "back"; // back, left, right
    String[] colorNames = {"No Ball", "Green", "Purple"};

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        led = hardwareMap.get(Servo.class, "led");

        // Initialize color sensors subsystem
        colorSensors = new ColorSensors_New(hardwareMap);

        telemetry.addLine("Color Sensor Test V2");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Cross (X) - Test Back sensors");
        telemetry.addLine("  Square - Test Right sensors");
        telemetry.addLine("  Circle - Test Left sensors");
        telemetry.addLine("  Right Bumper - Intake reverse");
        telemetry.addLine("  Left Bumper - Intake forward");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Intake control
            if (gamepad1.right_bumper) {
                intake.setPower(-1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Sensor selection
            if (gamepad1.cross) {
                sensorPosition = "back";
            }
            if (gamepad1.square) {
                sensorPosition = "right";
            }
            if (gamepad1.circle) {
                sensorPosition = "left";
            }

            // Get color detection results
            int combinedResult = 0;
            boolean potentialJam = false;

            if (sensorPosition.equals("left")) {
                combinedResult = colorSensors.getLeft();
                potentialJam = colorSensors.isPotentialJam("left");
                displaySensorDetails("LEFT 1", colorSensors.SensorLeft);
                displaySensorDetails("LEFT 2", colorSensors.SensorLeft2);
            } else if (sensorPosition.equals("right")) {
                combinedResult = colorSensors.getRight();
                potentialJam = colorSensors.isPotentialJam("right");
                displaySensorDetails("RIGHT 1", colorSensors.SensorRight);
                displaySensorDetails("RIGHT 2", colorSensors.SensorRight2);
            } else if (sensorPosition.equals("back")) {
                combinedResult = colorSensors.getBack();
                potentialJam = colorSensors.isPotentialJam("back");
                displaySensorDetails("BACK 1", colorSensors.SensorBack);
                displaySensorDetails("BACK 2", colorSensors.SensorBack2);
            }

            // Display combined result
            telemetry.addLine();
            telemetry.addLine("═══════════════════════════");
            telemetry.addData("POSITION", sensorPosition.toUpperCase());
            telemetry.addData("COMBINED RESULT", colorNames[combinedResult]);
            if (potentialJam) {
                telemetry.addLine("⚠️ POTENTIAL JAM DETECTED!");
            }
            telemetry.addLine("═══════════════════════════");

            // LED control based on combined result
            if (combinedResult == 1) {
                led.setPosition(0.5); // Green
            } else if (combinedResult == 2) {
                led.setPosition(0.72); // Purple
            } else {
                led.setPosition(0); // Off
            }

            telemetry.update();
        }
    }

    private void displaySensorDetails(String sensorName, NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;
        float totalIntensity = red + green + blue;

        // Calculate the blue/green ratio (key detection metric)
        float blueGreenRatio = (green > 0.001f) ? (blue / green) : 0f;

        // Get individual sensor color detection
        int detectedColor = colorSensors.getColor(sensor);

        telemetry.addLine();
        telemetry.addData(sensorName, colorNames[detectedColor]);
        telemetry.addData("  RGB", "R:%.3f G:%.3f B:%.3f", red, green, blue);
        telemetry.addData("  Total Intensity", "%.3f", totalIntensity);
        telemetry.addData("  Blue/Green Ratio", "%.3f", blueGreenRatio);

        // Visual indicator of ratio range
        if (blueGreenRatio < 0.75f && totalIntensity > 0.02f) {
            telemetry.addLine("  ✓ GREEN range (< 0.75)");
        } else if (blueGreenRatio > 1.08f && totalIntensity > 0.02f) {
            telemetry.addLine("  ✓ PURPLE range (> 1.08)");
        } else if (totalIntensity > 0.02f) {
            telemetry.addLine("  ⚠ AMBIGUOUS (0.75-1.08)");
        }
    }
}