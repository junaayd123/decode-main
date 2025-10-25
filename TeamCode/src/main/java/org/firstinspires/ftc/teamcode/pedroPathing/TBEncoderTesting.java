package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@Autonomous
public class TBEncoderTesting extends LinearOpMode{
    public CRServo sorterServo;
    NormalizedColorSensor colorSensor;
    float red;
    float green;
    float blue;
    int state;
    int CRStatus;

    DcMotor tbenc;
    public void runOpMode() throws InterruptedException{
        try {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            red = colors.red;
            green = colors.green;
            blue = colors.blue;
        } catch (Exception e){
            telemetry.addData("bruh", e);
        }
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterservo");
        double rotations; // this is a rotation based off of 1/3s
        int normalizedRotations;
        int normalizedSection;
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");
        waitForStart();
        while (!isStopRequested()&&opModeIsActive()) {
            rotations = ((double)-tbenc.getCurrentPosition()/(8192.0/3));
            normalizedRotations = (int)Math.floor(rotations);
            normalizedSection = Math.floorMod(normalizedRotations, 3);
            telemetry.addData("Encoder Data", tbenc.getCurrentPosition());
            telemetry.addData("Full rotation:", rotations);
            telemetry.addData("Full rotation:", normalizedRotations);
            telemetry.addData("Full rotation:", normalizedSection);


            // quadrant (or tridrant) = total rotations floor mod 3

            telemetry.update();
            if (red < 0.005 && green < 0.005 && blue < 0.005) {
                state = 0; // none
            }
            // --- Step 2: Detect Green vs Purple ---
            else if (green > blue && green > red) {
                state = 1; //green
            } else if (blue >= green && blue > red) {
                state = 2; //purple
            }
            telemetry.addData("State", state);
            if (gamepad1.a)
            {
                sorterServo.setPower(1);
            } else if (gamepad1.b) {
                sorterServo.setPower(-1);
            }
        }
    }
}
