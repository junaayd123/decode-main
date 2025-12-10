package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.A_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

@TeleOp(name = "ShakthiTeleOp", group = "TeleOp")
public class shakthiTeleOp extends OpMode {
    private boolean aligning = false;   // true while the robot is following the align path
//
//    private static final double FAR_BASE_POWER_12V   = 0.72;  // what worked for FAR at ~12.0V
//    private static final double FAR_BASE_POWER2_12V  = 0.715;  // a touch hotter between shots (optional)
//    private static final double CLOSE_BASE_POWER_12V = 0.55;  // what worked for CLOSE at ~12.0V

    private DcMotor intake = null;
    private Deposition depo;
    private boolean bluealliance = false;
    Gamepad preG1= new Gamepad();
//    public double curTime = 10000;
//    public double curTime2 = 10000;
//    public double curTime3 = 10000;
//    private ElapsedTime timer = new ElapsedTime();
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    Timer timer5;

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad g2= new Gamepad();
    private launch_lift LL;
    private double speed = 1;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        LL = new launch_lift(hardwareMap);
        follower = A_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = hardwareMap.get(DcMotor.class, "intake");
        depo = new Deposition(hardwareMap);
        timer1 = new Timer();
        timer2 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer5 = new Timer();

        g1.copy(gamepad1);
        g2.copy(gamepad2);

        // Optional: set directions if your motors are mounted opposite each other
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // depo.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        LL.down();
        LL.close();
        timer1.resetTimer();
        timer2.resetTimer();
        timer3.resetTimer();
        timer4.resetTimer();
        timer5.resetTimer();
        depo.setTargetVelocity(0);
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        depo.updatePID();
        //intake with gampad cross
        if(g1.cross){
            speed = 0.3;
        }
        else{speed=1;}
        boolean allTimersOff = !timer1.timerIsOn() && !timer2.timerIsOn() && !timer3.timerIsOn() && !timer4.timerIsOn() && !timer5.timerIsOn();
        if (g2.right_bumper && allTimersOff) {
            intake.setPower(-1.0);
        }else if (g2.left_bumper && !preG2.left_bumper && allTimersOff){
            timer5.startTimer();
        }
        else if(!timer1.timerIsOn() && !timer2.timerIsOn() && allTimersOff) {
            intake.setPower(0.0);
        }
        if(g1.right_bumper && !preG1.right_bumper){
            LL.up();
        }
        else if(g1.left_bumper && !preG1.left_bumper){
            LL.down();
        }
        if (g1.dpad_right &&!preG1.dpad_right) {
            LL.launchServo.setPosition(LL.launchServo.getPosition()+0.1);
        }
        else if (g1.dpad_left &&!preG1.dpad_left) {
            LL.launchServo.setPosition(LL.launchServo.getPosition()-0.1);
        }
        if(g1.square && !preG1.square){
            align_far_red(bluealliance);
        }
        if(g1.ps && !preG1.ps){
            bluealliance = !bluealliance;
        }

        // Depo with gamepad cirlce
        if (g2.circle && !preG2.circle && !g2.start) {
            timer2.startTimer();
        }
        if (g2.square && !preG2.square){
            timer1.startTimer();
        }
        if(g2.triangle && !preG2.triangle){
            timer3.stopTimer();
        }
        if(g2.cross && !preG2.cross){
            timer4.startTimer();
        }

        farshoot3x();
        closeshoot3x();
        farshoot();
        closeshoot();
        spitOut();

        followerstuff();

        // Telemetry
        telemetry.addData("is alliance blue?",bluealliance);
        telemetry.addData("Launch Position", LL.launchServo.getPosition());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Depo Power", depo.right.getPower());
//        telemetry.addData("time since run", timer.seconds()-curTime);
        telemetry.update();
    }

    private void align_far_red(boolean blue) {
        double angle;
        if (blue) {
            angle = 22.5;
        } else {
            angle = -22.5;
        }
        Pose cur = follower.getPose();
        Pose align = new Pose(cur.getX() -Math.cos(angle)*4, cur.getY()-Math.sin(angle)*4, cur.getHeading() + Math.toRadians(angle));

        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, align)))
                .setLinearHeadingInterpolation(cur.getHeading(), align.getHeading())
                .build();

        // start path asynchronously
        follower.followPath(first, false);
        aligning = true;

        // optional immediate update so follower starts processing now (harmless)
        follower.update();
    }
    public void spitOut(){
        if(timer5.checkAtSeconds(0)) {
            intake.setPower(0.3);
        }
        if(timer5.checkAtSeconds(0.1)) {
            timer5.stopTimer();
        }
    }
    public void farshoot(){
        if(timer1.checkAtSeconds(0)){
            depo.setTargetVelocity(depo.farVelo);
            LL.far();
        }
        if(timer1.checkAtSeconds(0.5)){
            LL.up();
        }
        if (timer1.checkAtSeconds(0.8)){
            LL.down();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
        }
    }


    public void closeshoot(){
        if(timer2.checkAtSeconds(0)){
            depo.setTargetVelocity(depo.closeVelo);
            LL.close();
        }
        if(timer2.checkAtSeconds(0.5)){
            LL.up();
        }
        if (timer2.checkAtSeconds(0.8)){
            LL.down();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
        }
    }
    public void farshoot3x() {
        // shot 1
        if (timer3.checkAtSeconds(0)) {
            depo.setTargetVelocity(depo.farVelo);
            LL.far();
        }
        if (timer3.checkAtSeconds(0.5)) LL.up();
        if (timer3.checkAtSeconds(0.8)) LL.down();

        // feed 1 and slightly bump power if you used to
        if (timer3.checkAtSeconds(1.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 2
        if (timer3.checkAtSeconds(1.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer3.checkAtSeconds(1.8)) LL.down();

        // feed 2 + return to main FAR power
        if (timer3.checkAtSeconds(2.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 3
        if (timer3.checkAtSeconds(2.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer3.checkAtSeconds(2.8)) {
            LL.down();
            depo.setTargetVelocity(0);
            timer3.stopTimer();
        }
    }

    public void closeshoot3x() {
        // shot 1
        if (timer4.checkAtSeconds(0)) {
            depo.setTargetVelocity(depo.closeVelo);
            LL.close();
        }
        if (timer4.checkAtSeconds(0.5)) LL.up();
        if (timer4.checkAtSeconds(0.8)) LL.down();

        // feed 1 and slightly bump power if you used to
        if (timer4.checkAtSeconds(1.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 2
        if (timer4.checkAtSeconds(1.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer4.checkAtSeconds(1.8)) LL.down();

        // feed 2 + return to main FAR power
        if (timer4.checkAtSeconds(2.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 3
        if (timer4.checkAtSeconds(2.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer4.checkAtSeconds(2.8)) {
            LL.down();
            depo.setTargetVelocity(0);
            timer4.stopTimer();
        }
    }
    private void followerstuff(){
        // Update follower first so it can progress following path if active
        follower.update();

// Telemetry & debug info about follower state
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("AligningFlag", aligning);

// If we are currently aligning, wait for follower to finish and then return to teleop
        if (aligning) {
            if (!follower.isBusy()) {
                // path finished
                follower.startTeleopDrive();  // re-enable teleop drive mode in follower
                aligning = false;
                telemetry.addData("AlignStatus", "Finished - teleop re-enabled");
            } else {
                // follower still busy — optionally skip applying driver stick control until finished
                telemetry.addData("AlignStatus", "Running");
            }
        }

// Only set teleop movement vectors when follower is NOT busy (i.e., manual control allowed)
        if (!follower.isBusy()) {
            // apply driver joystick input
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speed,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                    -gamepad1.right_stick_x * speed,
                    true
            );
        } else {
            // optional: zero sticks to make behavior predictable while follower drives
            // follower.setTeleOpMovementVectors(0, 0, 0, true);
        }

// Make sure to call update again only if needed — we already called it at top so this is fine
// follower.update();

    }

//    public void farshoot3x(){
//        if(timer.seconds() >= curTime2 && timer.seconds()<curTime2+0.1){
//            depo.shootFar();
//            LL.far();
//        }
//        if(timer.seconds() >= curTime2 +1.0 && timer.seconds()< curTime2+1.1){
//            LL.up();
//        }
//        if (timer.seconds()>=curTime2+1.3 && timer.seconds() < curTime2+1.4){
//            LL.down();
//        }
//        //intake 1
//        if (timer.seconds()>=curTime2+1.5 && timer.seconds() < curTime2+1.6){
//            intake.setPower(-1);
//            depo.shootFar2();
//        }
//        //shoot 2
//        if(timer.seconds() >= curTime2 +2 && timer.seconds()< curTime2+2.1){
//            LL.up();
////            intake.setPower(0);
//        }
//        if (timer.seconds()>=curTime2+2.3 && timer.seconds() < curTime2+2.4){
//            LL.down();
//        }
//        //intake 2
//        if (timer.seconds()>=curTime2+2.5 && timer.seconds() < curTime2+2.6){
//            intake.setPower(-1);
//            depo.shootFar();
//        }
//        //shoot 3
//        if(timer.seconds() >= curTime2 +3.0 && timer.seconds()< curTime2+3.1){
//            LL.up();
////            intake.setPower(0);
//        }
//        if (timer.seconds()>=curTime2+3.3 && timer.seconds() < curTime2+3.4){
//            LL.down();
//            intake.setPower(0);
//            depo.setPowerBoth(0.0);
//            curTime2 = 10000;
//        }
//
//    }
}
