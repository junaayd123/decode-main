package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Servo Tuner", group = "Tuning")
public class ServoTuner extends OpMode {
    private Servo[] servos;
    private String[] servoNames = { "lift_left","lift_right","lift_back","launch_angle" }; // Change to your servo names
    //    private String[] servoNames = { "VA", "VP", "VE","VR","VT" }; // Change to your servo names
    private int currentServoIndex = 0;
    private double[] positions;

    // For edge detection (prevent holding the button from rapid increment)
    private Gamepad prevGamepad1 = new Gamepad();
    private Gamepad currGamepad1 = new Gamepad();
    private boolean[] reversed;

    @Override
    public void init() {
        servos = new Servo[servoNames.length];
        positions = new double[servoNames.length];
        reversed = new boolean[servoNames.length];

        for (int i = 0; i < servoNames.length; i++) {
            servos[i] = hardwareMap.get(Servo.class, servoNames[i]);

            // Reverse specific servos from the start
            if (servoNames[i].equals("lift_left") || servoNames[i].equals("lift_back") || servoNames[i].equals("HE")|| servoNames[i].equals("VE")) {
                servos[i].setDirection(Servo.Direction.REVERSE);
                reversed[i] = true;
            } else {
                servos[i].setDirection(Servo.Direction.FORWARD);
                reversed[i] = false;
            }

            // Start all servos at 0 position
            if(servoNames[i].equals("VT")){
                positions[i] = 0.5;
            }
            else if(servoNames[i].equals("VE")){
                positions[i] = 0.15;
            }
            else {
                positions[i] = 0.0;
            }

            servos[i].setPosition(positions[i]);
        }
    }

    @Override
    public void loop() {
        // Update gamepad states
        prevGamepad1.copy(currGamepad1);
        currGamepad1.copy(gamepad1);

        // Cycle servo selection with bumpers
        if (currGamepad1.right_bumper && !prevGamepad1.right_bumper) {
            currentServoIndex = (currentServoIndex + 1) % servos.length;
        }
        if (currGamepad1.left_bumper && !prevGamepad1.left_bumper) {
            currentServoIndex = (currentServoIndex - 1 + servos.length) % servos.length;
        }

        // Adjust position with D-Pad
        if (currGamepad1.dpad_up && !prevGamepad1.dpad_up) {
            positions[currentServoIndex] = Math.min(1.0, positions[currentServoIndex] + 0.01);
        }
        if (currGamepad1.dpad_down && !prevGamepad1.dpad_down) {
            positions[currentServoIndex] = Math.max(0.0, positions[currentServoIndex] - 0.01);
        }
        if (currGamepad1.y && !prevGamepad1.y) {
            reversed[currentServoIndex] = !reversed[currentServoIndex];
            servos[currentServoIndex].setDirection(
                    reversed[currentServoIndex] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD
            );
        }

        // Apply positions to all servos
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(positions[i]);
        }

        // Telemetry
        telemetry.addData("Selected Servo", servoNames[currentServoIndex]);
        for (int i = 0; i < servoNames.length; i++) {
            telemetry.addData(servoNames[i], "%.2f", positions[i]);
        }
        telemetry.update();
    }
}