package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import android.service.autofill.DateValueSanitizer;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ColorSense_Test")
public class ColorSense_Test extends LinearOpMode {
    Servo led;
    DcMotor intake;
    String detectedColor = "Unknown";
    String sensorname = "back";
    int left1 = 0;
    int left2 = 0;
    int left3 = 0;
    int right1 = 0;
    int right2 = 0;
    int right3 = 0;
    int back1 = 0;
    int back2 = 0;
    int back3 = 0;
    float red;
    float green;
    float blue;
    public NormalizedColorSensor SensorLeft;
    public NormalizedColorSensor SensorRight;
    public NormalizedColorSensor SensorBack;
    public NormalizedColorSensor SensorLeft2;
    public NormalizedColorSensor SensorRight2;
    public NormalizedColorSensor SensorBack2;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        led = hardwareMap.get(Servo.class, "led");
        SensorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back");
        SensorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        SensorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right");
        SensorBack2 = hardwareMap.get(NormalizedColorSensor.class, "color_back2");
        SensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_left2");
        SensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_right2");
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                intake.setPower(-1);
            }
            else if(gamepad1.left_bumper){
                intake.setPower(1);
            }
            else intake.setPower(0);

//            telemetry.addData("left", left2);
//            telemetry.addData("right", right2);
//            telemetry.addData("back", back2);


            if(gamepad1.cross){
                sensorname = "back";
            }
            if(gamepad1.square){
                sensorname = "right";
            }
            if(gamepad1.circle){
                sensorname = "left";
            }
            telemetry.addData("sensor", sensorname);
            if(sensorname == "left"){
                left1 = detect(SensorLeft);
                telemetry.addLine("sensor 1");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
                left2 = detect(SensorLeft2);
                telemetry.addLine("sensor 2");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
            }
            if(sensorname == "right"){
                left1 = detect(SensorRight);
                telemetry.addLine("sensor 1");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
                left2 = detect(SensorRight2);
                telemetry.addLine("sensor 2");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
            }
            if(sensorname == "back"){
                left1 = detect(SensorBack);
                telemetry.addLine("sensor 1");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
                left2 = detect(SensorBack2);
                telemetry.addLine("sensor 2");
                telemetry.addData("Red", red);
                telemetry.addData("Green", green);
                telemetry.addData("Blue",  blue);
                telemetry.addData("Detected Color", detectedColor);
            }
            if(left1==1|| left2==1){
                led.setPosition(0.5);
            }
            else if(left1==2|| left2==2){
                led.setPosition(0.72);
            }
            else led.setPosition(0);


//            telemetry.addData("Red", red);
//            telemetry.addData("Green", green);
//            telemetry.addData("Blue",  blue);

//            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }
    public int detect(NormalizedColorSensor colorSensor){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        red = colors.red;
        green = colors.green;
        blue = colors.blue;



        // --- Step 1: Detect if no ball is present ---
        if (red < 0.005 && green < 0.005 && blue < 0.005) {
//            if(blue>=green){
//                detectedColor = "Purple Ball";
////                led.setPosition(0.72);
//                return 2;
//            }
//            else if(red*2<green){
//                detectedColor = "Green Ball";
//                led.setPosition(0.5);
//            }
//            else if(red<0.001 && green>0.0017 && green>blue)
//            {
//                detectedColor = "Green Ball";
//                led.setPosition(0.5);
//            }
//            else {
                detectedColor = "No Ball";
//                led.setPosition(0);
                return 0;
//            }
        }
        // --- Step 2: Detect Green vs Purple ---
        else if (green > blue && green > red) {
            detectedColor = "Green Ball";
//            led.setPosition(0.5);
            return 1;
        } else if (blue >= green && blue > red) {
            detectedColor = "Purple Ball";
//            led.setPosition(0.72);
            return 2;
        }
        else return 0;
    }
}
