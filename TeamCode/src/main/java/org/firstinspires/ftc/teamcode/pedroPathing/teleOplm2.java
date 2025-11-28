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
    Timer timer2;
    double ourVelo = 1300;
    boolean shooting = false;
    boolean shooting2 = false;
    boolean greenball = false;//false is purp true is geren

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
        timer2 = new Timer();

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
        timer2.resetTimer();

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
        if(g2.cross && !preG2.cross){//shoot 3 close
            if(!LL.checkNoBalls()) {
                depo.setTargetVelocity(depo.closeVelo_New);
                LL.set_angle_close();
                shooting = true;
            }
        }
        if(g2.square && !preG2.square){//shoot purp
            depo.setTargetVelocity(depo.closeVelo_New);
            LL.set_angle_close();
            greenball = false;
            shooting2 = true;
        }
        if(g2.circle && !preG2.circle){//shoot green
            depo.setTargetVelocity(depo.closeVelo_New);
            LL.set_angle_close();
            greenball = true;
            shooting2 = true;
        }
        if(g1.square && !preG1.square){
            direction = !direction;
        }
        if(g2.triangle && !preG2.triangle){//shoot far
            if(!LL.checkNoBalls()) {
                depo.setTargetVelocity(depo.farVelo_New);
                LL.set_angle_far();
                shooting = true;
            }
        }
        if(depo.reachedTarget()){
            if(shooting) {
                timer1.startTimer();//shoot3x function runs
                shooting = false;
            }
            if(shooting2){
                timer2.startTimer();
                shooting2 = false;
            }
        }
        shoot3x();
        shootoneColored();
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
            if(LL.sensors.getLeft()!=0) LL.leftUp();
            else if(LL.sensors.getRight()!=0) LL.rightUp();
            else if(LL.sensors.getBack()!=0) LL.backUp();
        }
        if(timer1.checkAtSeconds(0.3)){
            LL.allDown();
            if(LL.sensors.getLeft()!=0) LL.leftUp();
            else if(LL.sensors.getRight()!=0) LL.rightUp();
            else if(LL.sensors.getBack()!=0) LL.backUp();
            else{
                depo.setTargetVelocity(0);
                timer1.stopTimer();
            }
        }
        if(timer1.checkAtSeconds(0.6)){
            LL.allDown();
            if(LL.sensors.getLeft()!=0) LL.leftUp();
            else if(LL.sensors.getRight()!=0) LL.rightUp();
            else if(LL.sensors.getBack()!=0) LL.backUp();
            else{
                depo.setTargetVelocity(0);
                timer1.stopTimer();
            }
        }
        if(timer1.checkAtSeconds(0.9)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
        }

    }

    private void shootoneColored(){
        if(timer2.checkAtSeconds(0)){
            if(greenball){
                LL.lift_green();
            }
            else LL.lift_purple();
        }
        if(timer2.checkAtSeconds(0.3)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
        }
    }

}