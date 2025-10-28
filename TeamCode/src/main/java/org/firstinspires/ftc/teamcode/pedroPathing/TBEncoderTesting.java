package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;
import java.util.List;

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
        int normalizedEncoderValue;
        int degrees;
        int[] degreesModArray = new int[3];
        int[] degreeArray = {0, 120, 240};
        int degreesError = 40;
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");
        waitForStart();
        while (!isStopRequested()&&opModeIsActive()) {
            rotations = ((double)-tbenc.getCurrentPosition()/(8192.0/3));
            normalizedRotations = (int)Math.floor(rotations);
            normalizedEncoderValue = Math.floorMod(tbenc.getCurrentPosition(), 8192);
            normalizedSection = Math.floorMod(normalizedRotations, 3);
            degrees = (int)Math.floor((double)-tbenc.getCurrentPosition()/(8192.0/360.0));
            degreesModArray[0] = Math.floorMod(degrees, 360);
            degreesModArray[1] = Math.floorMod(degrees-120, 360);
            degreesModArray[2] = Math.floorMod(degrees-240, 360);


            //midpoint values: 0.5, 1.5, 2.5
            telemetry.addData("Encoder Data", tbenc.getCurrentPosition());
            telemetry.addData("Full rotation:", rotations);
            telemetry.addData("Full rotation Integer:", normalizedRotations);
            telemetry.addData("Full rotation Modulus:", normalizedSection);
            telemetry.addData("Degrees:", degrees);
            telemetry.addData("Degrees Modulus:", degreesModArray[0]);
            telemetry.addData("Degrees Modulus:", degreesModArray[1]);
            telemetry.addData("Degrees Modulus:", degreesModArray[2]);


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
                CRStatus = 1;
            } if (gamepad1.b) {
                CRStatus = 2;
            } if (gamepad1.x) {
                CRStatus = 0;
            } if (gamepad1.y) {
                CRStatus = -1;
            }
            if (CRStatus>=0) {
                if (CRStatus == 0) {
                    if (!(Math.abs(degreesModArray[0]) <degreesError) || !(Math.abs(degreesModArray[0]) >360-degreesError)) {
                        if (degreesModArray[0] < 360 - degreesModArray[0]) {
                            sorterServo.setPower(0.2);
                        } else {
                            sorterServo.setPower(-0.2);
                        }
                    } else {
                        CRStatus = -1;
                        sorterServo.setPower(0);
                    }
                }
                if (CRStatus == 1) {
                    if (!(Math.abs(degreesModArray[1]) <degreesError) || !(Math.abs(degreesModArray[1]) >360-degreesError)) {
                        if (degreesModArray[1] < 360 - degreesModArray[1]) {
                            sorterServo.setPower(0.2);
                        } else {
                            sorterServo.setPower(-0.2);
                        }
                    } else {
                        CRStatus = -1;
                        sorterServo.setPower(0);
                    }
                }
                if (CRStatus == 2) {
                    if (!(Math.abs(degreesModArray[2]) <degreesError) || !(Math.abs(degreesModArray[2]) >360-degreesError)) {
                        if (degreesModArray[2] < 360 - degreesModArray[2]) {
                            sorterServo.setPower(0.2);
                        } else {
                            sorterServo.setPower(-0.2);
                        }
                    } else {
                        CRStatus = -1;
                        sorterServo.setPower(0);
                    }
                }

            } else {
                CRStatus = -1;
                sorterServo.setPower(0);
            }


            // IN THE FUTURE, make a function to go to a COLOR, THEN FIND THE NEAREST COLOR THATS EASY.
        }
    }
}
