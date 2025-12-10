package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

@TeleOp(name="test flippers", group="Sensor")
public class testFlippers extends LinearOpMode {

    lift_three LL;
    DcMotor intake;

    Timer timer3;
    boolean intakeRunning;
    int whichServo; //0 left, 1 right 2 back


    @Override
    public void runOpMode() {
        LL = new lift_three(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        timer3 = new Timer();

        waitForStart();
        timer3.resetTimer();
        LL.allDown();
        while (opModeIsActive()) {
            if(gamepad1.dpadUpWasPressed()){
                if(whichServo ==0){
                    LL.liftLeft.setPosition(LL.liftLeft.getPosition()+0.01);
                }
                else if(whichServo ==1){
                    LL.liftRight.setPosition(LL.liftRight.getPosition()+0.01);
                }
                else if(whichServo ==2){
                    LL.liftBack.setPosition(LL.liftBack.getPosition()+0.01);
                }
            }else if(gamepad1.dpadDownWasPressed()){
                if(whichServo ==0){
                    LL.liftLeft.setPosition(LL.liftLeft.getPosition()-0.01);
                }
                else if(whichServo ==1){
                    LL.liftRight.setPosition(LL.liftRight.getPosition()-0.01);
                }
                else if(whichServo ==2){
                    LL.liftBack.setPosition(LL.liftBack.getPosition()-0.01);
                }
            }
            if(gamepad1.triangleWasPressed()){
                if(LL.liftBack.getPosition()>0) LL.backDown();
                else LL.backUp();
            }
            if(gamepad1.circleWasPressed()){
                if(LL.liftLeft.getPosition()>0) LL.leftDown();
                else LL.leftUp();
            }
            if(gamepad1.squareWasPressed()){
                if(LL.liftRight.getPosition()>0) LL.rightDown();
                else LL.rightUp();
            }
            if(gamepad1.aWasPressed()){
                if(whichServo ==2) whichServo = 0;
                else whichServo++;
            }
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
                    timer3.startTimer();
                    intakeRunning = false;
                }
            }
            telemetry.addData("Servo controlled",whichServo==0?"left":whichServo==1?"right":"back");
            telemetry.addData("left lift pos",LL.liftLeft.getPosition());
            telemetry.addData("right lift pos",LL.liftRight.getPosition());
            telemetry.addData("back lift pos",LL.liftBack.getPosition());
            telemetry.update();

            reverseIntake();


        }



    }

    private void reverseIntake() {
        if (timer3.checkAtSeconds(0)) {
            intake.setPower(1);
        }
        if (timer3.checkAtSeconds(0.5)) {
            intake.setPower(0);
            timer3.stopTimer();
        }
    }

}
