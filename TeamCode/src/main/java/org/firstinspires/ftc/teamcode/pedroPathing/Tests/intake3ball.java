package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

@TeleOp(name="intake3ball", group="Sensor")
public class intake3ball extends LinearOpMode {

    lift_three LL;
    DcMotor intake;

    Timer timer1;
    boolean intakeRunning;


    @Override
    public void runOpMode() {
        LL = new lift_three(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        timer1 = new Timer();

        waitForStart();
        timer1.resetTimer();

        while (opModeIsActive()) {

            if (gamepad1.leftBumperWasPressed()) {
                if (intake.getPower() < -0.5) {
                    intake.setPower(0);
                    intakeRunning = false;
                } else {
                    intake.setPower(-1);
                    intakeRunning = true;
                }
            }

            if (intakeRunning) {
                if (LL.sensors.getRight() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getLeft() != 0) {
                    timer1.startTimer();
                    intakeRunning = false;
                }
            }



            reverseIntake();


        }



    }

    private void reverseIntake() {
        if (timer1.checkAtSeconds(0)) {
            intake.setPower(1);
        }
        if (timer1.checkAtSeconds(0.5)) {
            intake.setPower(0);
            timer1.stopTimer();
        }
    }

}
