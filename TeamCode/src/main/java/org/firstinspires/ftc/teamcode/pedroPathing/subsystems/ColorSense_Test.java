package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorSense_Test")
public class ColorSense_Test extends LinearOpMode {

    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;

            String detectedColor = "Unknown";

            // --- Step 1: Detect if no ball is present ---
            if (red < 0.005 && green < 0.005 && blue < 0.005) {
                detectedColor = "No Ball";
            }
            // --- Step 2: Detect Green vs Purple ---
            else if (green > blue && green > red) {
                detectedColor = "Green Ball";
            } else if (blue >= green && blue > red) {
                detectedColor = "Purple Ball";
            }

            telemetry.addData("Red", "%.3f", red);
            telemetry.addData("Green", "%.3f", green);
            telemetry.addData("Blue", "%.3f", blue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }
}
