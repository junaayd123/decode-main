package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Lm1TeleOp", group = "TeleOp")
public class simple_lm1_teleop extends OpMode {
    private DcMotor intake = null;
    private Deposition depo;
    private lift_three LL;
    Timer timer1;
    double ourVelo = 1300;
    boolean shooting = false;

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2= new Gamepad();

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        LL = new lift_three(hardwareMap);
        depo = new Deposition(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        timer1 = new Timer();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Optional: set directions if your motors are mounted opposite each other
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // depo.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        depo.setTargetVelocity(0);
        depo.kF = -0.00048;
        LL.allDown();
        LL.launchAngleServo.setPosition(0);
        timer1.resetTimer();
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();
        if(g1.right_bumper){
            intake.setPower(-1);
        } else if (g1.left_bumper) {
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
        if(g1.cross && !preG1.cross){
            if(LL.liftBack.getPosition()<0.1) LL.backUp();
            else LL.backDown();
        }
        if(g1.circle && !preG1.circle){
            if(LL.liftRight.getPosition()<0.1) LL.rightUp();
            else LL.rightDown();
        }
        if(g1.square && !preG1.square){
            if(LL.liftLeft.getPosition()<0.1) LL.leftUp();
            else LL.leftDown();
        }
        if(g1.triangle && !preG1.triangle){
//            if(depo.getVelocity()<50 && depo.getVelocity()>-50) depo.setTargetVelocity(ourVelo);
//            else depo.setTargetVelocity(0);
            depo.setTargetVelocity(ourVelo);
            shooting = true;
        }
        if(shooting && depo.reachedTarget()){
            timer1.startTimer();
            shooting =false;
        }
        shoot3x();
        if(g1.dpad_up&& !preG1.dpad_up){
            ourVelo+=50;
        }
        else if(g1.dpad_down&& !preG1.dpad_down){
            ourVelo-=50;
        }
        if(g1.dpad_left&& !preG1.dpad_left){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()-0.03);
        }
        else if(g1.dpad_right&& !preG1.dpad_right){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()+0.03);
        }



        follower.setTeleOpDrive( -gamepad1.left_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x, true );
        follower.update();

        // Telemetry
        telemetry.addData("target velocity", ourVelo);
        telemetry.update();
    }
    private void shoot3x(){
        if(timer1.checkAtSeconds(0)){
            LL.leftUp();
        }
        if(timer1.checkAtSeconds(0.3)){
            LL.leftDown();
            LL.rightUp();
        }
        if(timer1.checkAtSeconds(0.6)){
            LL.rightDown();
            LL.backUp();
        }
        if(timer1.checkAtSeconds(0.9)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
        }

    }

}