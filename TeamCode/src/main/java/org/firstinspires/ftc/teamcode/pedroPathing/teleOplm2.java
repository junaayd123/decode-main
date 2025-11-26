package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

@TeleOp(name = "Lm2 2 drivers", group = "TeleOp")
public class teleOplm2 extends OpMode {
    private DcMotor intake = null;
    private Deposition depo;
    Servo led;
    private lift_three LL;
    boolean direction = false; //false if intake is forward, true if depo;
    double speed;
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
        led = hardwareMap.get(Servo.class, "led");
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
        LL.set_angle_min();
        timer1.resetTimer();
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();
        if(g2.right_bumper){
            intake.setPower(-1);
        } else if (g2.left_bumper) {
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
        if(g1.cross) speed = 0.3;
        else speed = 1;
        if(g2.cross && !preG2.cross){
            depo.setTargetVelocity(depo.closeVelo_New);
            LL.set_angle_close();
            shooting = true;
//            if(LL.liftRight.getPosition()<0.1) LL.rightUp();
//            else LL.rightDown();
        }
        if(g1.square && !preG1.square){
//            if(LL.liftLeft.getPosition()<0.1) LL.leftUp();
//            else LL.leftDown();
            direction = !direction;
        }
        if(g2.triangle && !preG2.triangle){
//            if(depo.getVelocity()<50 && depo.getVelocity()>-50) depo.setTargetVelocity(ourVelo);
//            else depo.setTargetVelocity(0);
            depo.setTargetVelocity(depo.farVelo_New);
            LL.set_angle_far();
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


        if(direction) {
            led.setPosition(0.388);
            follower.setTeleOpDrive( gamepad1.left_stick_y*speed, (gamepad1.right_trigger - gamepad1.left_trigger)*speed, -gamepad1.right_stick_x*speed, true );
        }
        else {
            led.setPosition(0);
            follower.setTeleOpDrive( -gamepad1.left_stick_y*speed, (gamepad1.left_trigger - gamepad1.right_trigger)*speed, -gamepad1.right_stick_x*speed, true );
        }
        follower.update();

        // Telemetry
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("curent angle", LL.launchAngleServo.getPosition());
        telemetry.addData("depo is forward",direction);
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