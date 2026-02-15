package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

@TeleOp(name = "ColorSense_Test", group = "z")
public class ColorSense_Test extends LinearOpMode {
    Servo led;
    Servo led2;
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

    Timer timer1;
    boolean intaking;

    @Override
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        led = hardwareMap.get(Servo.class, "led");
        led2 = hardwareMap.get(Servo.class, "led2");
        SensorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back");
        SensorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        SensorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right");
        SensorBack2 = hardwareMap.get(NormalizedColorSensor.class, "color_back2");
        SensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_left2");
        SensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_right2");
        timer1 = new Timer();
        waitForStart();
        timer1.resetTimer();
        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                intake.setPower(-1);
                intaking = false;
                timer1.stopTimer();
            }
            else if(gamepad1.left_bumper){
                intake.setPower(1);
            }
            else if(!intaking && timer1.timerIsOff()) intake.setPower(0);

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
            left1 = detect(SensorLeft);
            left2 = detect(SensorLeft2);
            right1 = detect(SensorRight);
            right2 = detect(SensorRight2);
            back1 = detect(SensorBack);
            back2 = detect(SensorBack2);

            if(left1==1|| left2==1) led.setPosition(0.5);
            else if(left1==2|| left2==2) led.setPosition(0.72);
            else led.setPosition(0);

            if(right1==1|| right2==1) led2.setPosition(0.5);
            else if(right1==2|| right2==2) led2.setPosition(0.72);
            else led2.setPosition(0);

            if(back1==1|| back2==1) gamepad1.setLedColor(0,255,0,100);
            else if(back1==2|| back2==2) gamepad1.setLedColor(255,0,255,100);
            else gamepad1.setLedColor(0,0,0,100);
            if(gamepad1.dpadDownWasPressed()){
                intaking = true;
                intake.setPower(-1);
            }
            if(intaking){
                if((back1!=0|| back2!=0) && (right1!=0|| right2!=0) && (left1!=0|| left2!=0) ){
                    timer1.startTimer();
                    intake.setPower(1);
                    intaking = false;
                }

            }
            if(timer1.checkAtSeconds(3)) timer1.stopTimer();

//            telemetry.addData("Red", red);
//            telemetry.addData("Green", green);
//            telemetry.addData("Blue",  blue);

//            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }
    public int detect(NormalizedColorSensor colorSensor){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        red = colors.red*1000;
        green = colors.green*1000;
        blue = colors.blue*1000;



        // --- Step 1: Detect if no ball is present ---
        if (red < 7 && green < 7 && blue < 7) {
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
