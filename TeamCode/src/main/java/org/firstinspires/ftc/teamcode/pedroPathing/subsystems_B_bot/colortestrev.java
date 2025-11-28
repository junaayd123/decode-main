package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ColorSenser test rev")
public class colortestrev extends LinearOpMode {
    Servo led;

    String detectedColor = "Unknown";
    String sensorname = "back";
    int red;
    int green;
    int blue;
    public RevColorSensorV3 SensorLeft;
    public RevColorSensorV3 SensorRight;
    public RevColorSensorV3 SensorBack;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            led = hardwareMap.get(Servo.class, "led");
            SensorBack = hardwareMap.get(RevColorSensorV3.class, "color_back");
            SensorLeft = hardwareMap.get(RevColorSensorV3.class, "color_left");
            SensorRight = hardwareMap.get(RevColorSensorV3.class, "color_right");
            if(gamepad1.cross){
                sensorname = "back";
            }
            if(gamepad1.square){
                sensorname = "right";
            }
            if(gamepad1.circle){
                sensorname = "left";
            }
            if(sensorname == "left"){
                detect(SensorLeft);
            }
            if(sensorname == "right"){
                detect(SensorRight);
            }
            if(sensorname == "back"){
                detect(SensorBack);
            }

            telemetry.addData("sensor", sensorname);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue",  blue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }
    public void detect(RevColorSensorV3 colorSensor){

        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();



        // --- Step 1: Detect if no ball is present ---
        if (red < 200 && green < 200 && blue < 200) {
            detectedColor = "No Ball";
            led.setPosition(0);
        }
        // --- Step 2: Detect Green vs Purple ---
        else if (green > blue && green > red) {
            detectedColor = "Green Ball";
            led.setPosition(0.5);
        } else if (blue >= green && blue > red) {
            detectedColor = "Purple Ball";
            led.setPosition(0.72);
        }
    }
}
