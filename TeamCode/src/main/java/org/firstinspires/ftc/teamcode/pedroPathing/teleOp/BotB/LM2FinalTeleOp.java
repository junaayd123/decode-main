package org.firstinspires.ftc.teamcode.pedroPathing.teleOp.BotB;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

@TeleOp(name = "LM2 Final teleOp", group = "TeleOp")
public class LM2FinalTeleOp extends OpMode {
    private boolean aligning = false;
    private boolean aligning2 = false;
    private boolean alignForFar = false;
    private double distanceToGoal;

    private boolean bluealliance = false;
    private double desiredHeading = 0;
    String motif = "gpp";

    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(320.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(220.0));
    private final Pose blueFarShootPose = new Pose(65, 25, Math.toRadians(-61));
    private final Pose redFarShootPose  = new Pose(80, 25, Math.toRadians(-115));
    private final Pose redGoal  = new Pose(144, 144, 0);
    private final Pose blueGoal  = new Pose(0, 144,0);
    private final Pose redGoal2  = new Pose(144, 144, 0);
    private final Pose blueGoal2  = new Pose(0, 144,0);
    private final Pose redHP  = new Pose(42, 25, Math.toRadians(180));
    private final Pose blueHP  = new Pose(115, 25,0);

    String shotSlot1;
    String shotSlot2;
    String shotSlot3;

    int[] ballsInRobot = {0,0,0};
    int greenInSlot;
    private DcMotor intake = null;
    private Deposition depo;
    boolean shootingTest =false;
    boolean intakeRunning;

    private lift_three LL;
    boolean direction = false;
    double speed;
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    Timer timer6;
    double ourVelo = 1300;
    boolean shooting = false;
    boolean shooting2 = false;

    // NEW: Delay variables for depo spin-up
    boolean waitingToShoot1 = false;
    boolean waitingToShoot6 = false;
    double depoSpinUpTime1 = 0;
    double depoSpinUpTime6 = 0;

    // Separate sequence variables for each timer
    int shooterSequence1;
    int shooterSequence2;
    int shooterSequence6;

    // Separate timing variables
    double timeOfSecondShot1;
    double timeOfSecondShot2;
    double timeOfSecondShot6;

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
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        timer1 = new Timer();
        timer2 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer6 = new Timer();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
        depo.setTargetVelocity(0);
        depo.kF = 0.00048;
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        timer3.resetTimer();
        timer4.resetTimer();
        timer6.resetTimer();
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();

        if(g1.psWasPressed()) bluealliance = !bluealliance;

        // ========= INTAKE CONTROL =========
        if (gamepad2.rightBumperWasPressed()) {
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

        if(g2.left_bumper){
            intake.setPower(1);
        }
        else if(!g2.left_bumper && !intakeRunning && !timer3.timerIsOn()){
            intake.setPower(0);
        }

        reverseIntake();

        // ========= SPEED CONTROL =========
        if(g1.right_bumper) speed = 0.3;
        else speed = 1;

        // ========= SHOOTING MODE TOGGLE =========
        if(g2.dpad_down && !preG2.dpad_down){
            shootingTest = !shootingTest;
        }

        // ========= SHOOTING TRIGGERS =========
        if(g2.cross && !preG2.cross){
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                }
                else {
                    depo.setTargetVelocity(1000);
                    LL.set_angle_close();
                    motif = "gpp";
                    greenInSlot = 0;
                }
                shooting = true;
            }
        }

        if(g1.triangle && !preG1.triangle) {
            depo.setTargetVelocity(1000);  // Changed to positive 1000
            LL.set_angle_close();
            shooting2 = true;
        }

        if(g2.square && !preG2.square){
            depo.setTargetVelocity(1000);
            LL.set_angle_close();
            shooting = true;
            motif = "gpp";
            greenInSlot = getGreenPos();
            ballsInRobot[0] = LL.sensors.getLeft();
            ballsInRobot[1] = LL.sensors.getRight();
            ballsInRobot[2] = LL.sensors.getBack();
        }

        // ========= DIRECTION TOGGLE =========
        if (g1.left_bumper && !preG1.left_bumper) {
            direction = !direction;
        }

        // ========= AUTO ALIGNMENT =========
        if (g1.circle && !preG1.circle && !follower.isBusy()) {
            if(distanceToGoal<115) {
                faceAllianceGoal();
                timer4.startTimer();
            }
            else{
                goToFarPose();
                alignForFar=true;
            }
        }

        if(alignForFar && !follower.isBusy()){
            timer4.startTimer();
            alignForFar = false;
        }

        quitCorrectingAngle();

        // ========= FORCE TELEOP CONTROL =========
        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            aligning = false;
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }

        followerstuff();

        // ========= TELEMETRY =========
        telemetry.addData("Alliance Blue?", bluealliance);
        Pose cur = follower.getPose();
        distanceToGoal = getDistance();
        telemetry.addData("first shot", shotSlot1);
        telemetry.addData("second shot", shotSlot2);
        telemetry.addData("third shot", shotSlot3);
        telemetry.addData("shooter sequence 1", shooterSequence1);
        telemetry.addData("shooter sequence 6", shooterSequence6);
        telemetry.addData("timer6 on?", timer6.timerIsOn());
        telemetry.addData("shooting2 flag", shooting2);
        telemetry.addData("waitingToShoot1", waitingToShoot1);
        telemetry.addData("waitingToShoot6", waitingToShoot6);
        telemetry.addData("actual depo velo",depo.getVelocity());
        telemetry.addData("target depo velo",depo.getTargetVelocity());
        telemetry.addLine(shootingTest ? "Testing shooting using cross":"regular teleOp shooting");
        telemetry.addData("distance to goal",distanceToGoal);
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("current angle", LL.launchAngleServo.getPosition());
        telemetry.addData("X", cur.getX());
        telemetry.addData("Y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("desired heading",Math.toDegrees(desiredHeading));

        // ========= START SHOOTING SEQUENCES WITH DELAY =========
        if(depo.reachedTargetHighTolerance()){
            if (shooting && !waitingToShoot1) {
                waitingToShoot1 = true;
                depoSpinUpTime1 = getRuntime();
                shooting = false;
            }
            if(shooting2 && !waitingToShoot6){
                waitingToShoot6 = true;
                depoSpinUpTime6 = getRuntime();
                shooting2 = false;
            }
        }

        // Start timer1 after 1 second delay
        if(waitingToShoot1 && (getRuntime() - depoSpinUpTime1 >= 1.0) && !timer1.timerIsOn()){
            timer1.startTimer();
            waitingToShoot1 = false;
        }

        // Start timer6 after 1 second delay
        if(waitingToShoot6 && (getRuntime() - depoSpinUpTime6 >= 1.0) && !timer6.timerIsOn()){
            timer6.startTimer();
            waitingToShoot6 = false;
        }

        // ========= EXECUTE MOTIF-BASED SHOOTING (TIMER 1) =========
        if(timer1.timerIsOn()) {
            if(motif.equals("gpp")){
                if(greenInSlot == 0) shootLRB();
                else if(greenInSlot == 1) shootRBL();
                else shootBLR();
            }
            else if(motif.equals("pgp")){
                if(greenInSlot == 0) shootBLR();
                else if(greenInSlot == 1) shootLRB();
                else shootRBL();
            }
            else{
                if(greenInSlot == 0) shootRBL();
                else if(greenInSlot == 1) shootBLR();
                else shootLRB();
            }
        }

        // ========= EXECUTE RANDOM SHOOTING SEQUENCE (TIMER 6) =========
        if(timer6.timerIsOn()) {
            shootThreeRandom();
        }

        // ========= MANUAL ADJUSTMENTS =========
        if(g1.dpad_up && !preG1.dpad_up){
            ourVelo+=25;
        }
        else if(g1.dpad_left && !preG1.dpad_left){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()-0.03);
        }
        else if(g1.dpad_right && !preG1.dpad_right){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()+0.03);
        }

        telemetry.update();
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

    private double getDistance(){
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal2 : redGoal2;
        double x = target.getX()-cur.getX();
        double y = target.getY()-cur.getY();
        double hypotenuse = Math.pow(x,2)+Math.pow(y,2);
        return Math.sqrt(hypotenuse);
    }

    private void goToFarPose() {
        Pose target = bluealliance ? blueFarShootPose : redFarShootPose;
        Pose cur = follower.getPose();

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();

        follower.followPath(chain);
    }

    private void faceAllianceGoal() {
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal : redGoal;

        double rawAngle = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());
        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        desiredHeading = flippedAngle;
        follower.turnTo(flippedAngle);
        aligning = true;
    }

    private void quitCorrectingAngle(){
        if(timer4.checkAtSeconds(1)){
            follower.breakFollowing();
            follower.startTeleopDrive();
            aligning = false;
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
            timer4.stopTimer();
        }
    }

    private void followerstuff() {
        follower.update();
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("AligningFlag", aligning);

        if (!follower.isBusy() && aligning2) {
            follower.startTeleopDrive();
            aligning2 = false;
            telemetry.addData("AlignStatus", "Finished - teleop re-enabled");
        } else if (aligning) {
            telemetry.addData("AlignStatus", "Running");
        }

        if (!follower.isBusy() && !aligning) {
            if(direction) {
                follower.setTeleOpDrive(gamepad1.left_stick_y*speed, (gamepad1.right_trigger - gamepad1.left_trigger)*speed, -gamepad1.right_stick_x*speed, true);
            }
            else {
                follower.setTeleOpDrive(-gamepad1.left_stick_y*speed, (gamepad1.left_trigger - gamepad1.right_trigger)*speed, -gamepad1.right_stick_x*speed, true);
            }
        }
    }

    private int getGreenPos(){
        int pos = LL.sensors.getLeft();
        if(pos==1) return 0;
        else{
            pos = LL.sensors.getRight();
            if(pos==1) return 1;
            else return 2;
        }
    }

    // ========= SHOOTING SEQUENCES - TIMER 1 =========
    private void shootLRB() {
        if (timer1.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence1 = 1;
        }

        if (timer1.checkAtSeconds(0.4) && shooterSequence1==1) {
            LL.allDown();
            shooterSequence1 = 2;
        }

        if(shooterSequence1==2 && depo.reachedTargetHighTolerance()){
            LL.rightUp();
            shooterSequence1=3;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==3) {
            LL.allDown();
            shooterSequence1 = 4;
        }

        if(shooterSequence1==4 && depo.reachedTargetHighTolerance()){
            LL.backUp();
            shooterSequence1=5;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence1 = 0;
        }
    }

    private void shootBLR(){
        if (timer1.checkAtSeconds(0)) {
            LL.backUp();
            shooterSequence1 = 1;
        }

        if (timer1.checkAtSeconds(0.4) && shooterSequence1==1) {
            LL.allDown();
            shooterSequence1 = 2;
        }

        if(shooterSequence1==2 && depo.reachedTargetHighTolerance()){
            LL.leftUp();
            shooterSequence1=3;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==3) {
            LL.allDown();
            shooterSequence1 = 4;
        }

        if(shooterSequence1==4 && depo.reachedTargetHighTolerance()){
            LL.rightUp();
            shooterSequence1=5;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence1 = 0;
        }
    }

    private void shootRBL(){
        if (timer1.checkAtSeconds(0)) {
            LL.rightUp();
            shooterSequence1 = 1;
        }

        if (timer1.checkAtSeconds(0.4) && shooterSequence1==1) {
            LL.allDown();
            shooterSequence1 = 2;
        }

        if(shooterSequence1==2 && depo.reachedTargetHighTolerance()){
            LL.backUp();
            shooterSequence1=3;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==3) {
            LL.allDown();
            shooterSequence1 = 4;
        }

        if(shooterSequence1==4 && depo.reachedTargetHighTolerance()){
            LL.leftUp();
            shooterSequence1=5;
            timeOfSecondShot1 = timer1.timer.seconds()-timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4+timeOfSecondShot1) && shooterSequence1==5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence1 = 0;
        }
    }

    // ========= RANDOM SHOOTING SEQUENCE - TIMER 6 =========
    private void shootThreeRandom() {
        if (timer6.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence6 = 1;
        }

        if (timer6.checkAtSeconds(0.4) && shooterSequence6 == 1) {
            LL.allDown();
            shooterSequence6 = 2;
        }

        if(shooterSequence6 == 2 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            shooterSequence6 = 3;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }

        if (timer6.checkAtSeconds(0.4 + timeOfSecondShot6) && shooterSequence6 == 3) {
            LL.allDown();
            shooterSequence6 = 4;
        }

        if(shooterSequence6 == 4 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            shooterSequence6 = 5;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }

        if (timer6.checkAtSeconds(0.4 + timeOfSecondShot6) && shooterSequence6 == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer6.stopTimer();
            shooterSequence6 = 0;
        }
    }
}