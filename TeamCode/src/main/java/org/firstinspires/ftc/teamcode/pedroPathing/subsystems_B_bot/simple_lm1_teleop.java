package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Ball Detector Light Program", group="Sensor")
public class simple_lm1_teleop extends LinearOpMode {

    private NormalizedColorSensor colorLeft, colorRight, colorBack;
    private Servo led;
    private DcMotor intakeMotor;

    // intake toggle
    private boolean intakeOn = false;
    private boolean previousA = false;

    // Reverse intake for unjamming
    private boolean reversing = false;
    private ElapsedTime reverseTimer = new ElapsedTime();

    // Temporary holders for classifyBall()
    int purpleDetected = 0;
    int greenDetected = 0;

    @Override
    public void runOpMode() {

        // Initialize sensors safely
        try { colorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left"); }
        catch (Exception e) { telemetry.addData("color_left", "Not Found"); }

        try { colorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right"); }
        catch (Exception e) { telemetry.addData("color_right", "Not Found"); }

        try { colorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back"); }
        catch (Exception e) { telemetry.addData("color_back", "Not Found"); }

        try { led = hardwareMap.get(Servo.class, "led"); }
        catch (Exception e) { telemetry.addData("led", "Not Found"); }

        // Intake motor initialization
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("intakeMotor", "Not Found");
        }

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ------------------------
            // BUTTON TOGGLE LOGIC
            // ------------------------
            if (gamepad1.a && !previousA) {
                intakeOn = !intakeOn; // toggle intake
            }
            previousA = gamepad1.a;

            // ------------------------
            // COUNT BALLS
            // ------------------------
            int purpleCount = 0;
            int greenCount = 0;
            int ballCount = 0;

            ballCount += classifyBall(colorLeft);
            purpleCount += purpleDetected;
            greenCount += greenDetected;

            ballCount += classifyBall(colorRight);
            purpleCount += purpleDetected;
            greenCount += greenDetected;

            ballCount += classifyBall(colorBack);
            purpleCount += purpleDetected;
            greenCount += greenDetected;

            // ------------------------
            // AUTOMATIC INTAKE LOGIC
            // ------------------------
            if (intakeMotor != null) {
                if (reversing) {
                    intakeMotor.setPower(-1); // reverse intake
                    if (reverseTimer.seconds() >= 2.0) {
                        intakeMotor.setPower(0); // stop after 2 seconds
                        reversing = false;
                        intakeOn = false; // reset toggle
                    }
                } else if (intakeOn && ballCount >= 3) {
                    reversing = true;
                    reverseTimer.reset();
                } else if (intakeOn) {
                    intakeMotor.setPower(1.0); // normal intake
                } else {
                    intakeMotor.setPower(0); // intake off
                }
            }

            // ------------------------
            // LED LOGIC
            // ------------------------
            if (led != null) {
                if (ballCount == 0) {
                    led.setPosition(1.0);   // WHITE
                } else if (purpleCount == 2 && greenCount == 1) {
                    led.setPosition(0.5);   // GREEN = PPG
                } else {
                    led.setPosition(0.277); // RED = anything else
                }
            }

            telemetry.addData("Purple Count", purpleCount);
            telemetry.addData("Green Count", greenCount);
            telemetry.addData("Total Balls", ballCount);
            telemetry.addData("Intake On", intakeOn);
            telemetry.addData("Reversing", reversing);
            telemetry.update();
        }
    }

    // ====================================================
    // classifyBall() with improved tolerance
    // ====================================================
    private int classifyBall(NormalizedColorSensor sensor) {

        purpleDetected = 0;
        greenDetected = 0;

        if (sensor == null) return 0;

        NormalizedRGBA c = sensor.getNormalizedColors();
        float brightness = c.red + c.green + c.blue;

        // ---------- BALL PRESENCE ----------
        if (brightness < 0.08) {
            return 0; // no ball
        }

        // ---------- PURPLE DETECTION ----------
        if ((c.red + c.blue) > (c.green * 1.15)) {
            purpleDetected = 1;
            return 1;
        }

        // ---------- GREEN DETECTION ----------
        if ((c.green > c.red * 1.1) && (c.green > c.blue * 1.1)) {
            greenDetected = 1;
            return 1;
        }

        // ---------- AMBIGUOUS → default to green ----------
        greenDetected = 1;
        return 1;
    }
}
