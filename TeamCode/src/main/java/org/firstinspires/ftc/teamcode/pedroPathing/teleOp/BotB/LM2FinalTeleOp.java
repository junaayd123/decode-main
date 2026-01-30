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

    private Pose homePose = null;
    private boolean returningHome = false;
    private double arrivedAtHomeTime = 0;
    private boolean waitingAtHome = false;

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
    boolean shootingTest = false;
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

    private int loopCounter = 0;

    boolean waitingToShoot1 = false;
    boolean waitingToShoot6 = false;
    double depoSpinUpTime1 = 0;
    double depoSpinUpTime6 = 0;

    int shooterSequence1;
    int shooterSequence2;
    int shooterSequence6;

    double timeOfSecondShot1;
    double timeOfSecondShot2;
    double timeOfSecondShot6;

    Gamepad g1 = new Gamepad();
    Gamepad preG2 = new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2 = new Gamepad();

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
        LL.resetBallCount();
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

        if(loopCounter++ >= 5) {
            LL.updateBallTracking();
            loopCounter = 0;
        }

        if(g1.psWasPressed()) bluealliance = !bluealliance;

        // ========= HOME POSITION CONTROL =========
        if(g1.dpad_right && !preG1.dpad_right) {
            Pose currentPose = follower.getPose();
            homePose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
            follower.setPose(new Pose(0, 0, 0));
            telemetry.addLine(">>> HOME POSITION SET <<<");
        }

        // FIX #2: Use PathChain to return to home instead of manual control
        // Toggle return to home - press triangle again to cancel
        if(g1.triangle && !preG1.triangle && homePose != null) {
            if(returningHome || waitingAtHome) {
                // Cancel return to home
                follower.breakFollowing();
                follower.startTeleopDrive();
                returningHome = false;
                waitingAtHome = false;
                telemetry.addLine(">>> RETURN TO HOME CANCELLED <<<");
            } else if(!follower.isBusy()) {
                // Start return to home
                returnToHome();
            }
        }

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

        if (intakeRunning && LL.getBallCount() >= 3) {
            timer3.startTimer();
            intakeRunning = false;
        }

        if(g2.left_bumper){
            intake.setPower(1);
        }
        else if(!g2.left_bumper && !intakeRunning && !timer3.timerIsOn()){
            intake.setPower(0);
        }

        reverseIntake();

        speed = g1.right_bumper ? 0.3 : 1;

        if(g2.dpad_down && !preG2.dpad_down){
            shootingTest = !shootingTest;
        }

        // ─── Shooting triggers ───────────────────────────────────────
        // FIX #1: Prevent shooting triggers from being pressed while already shooting
        if(g2.triangle && !preG2.triangle && !timer6.timerIsOn() && !waitingToShoot6) {
            depo.setTargetVelocity(1300);
            LL.set_angle_close();
            shooting2 = true;
        }

        if(g2.square && !preG2.square && !timer6.timerIsOn() && !waitingToShoot6) {
            depo.setTargetVelocity(1800);
            LL.set_angle_far_auto();
            shooting2 = true;
        }

        if (g1.left_bumper && !preG1.left_bumper) {
            direction = !direction;
        }

        if (g1.circle && !preG1.circle && !follower.isBusy()) {
            if(distanceToGoal < 115) {
                faceAllianceGoal();
                timer4.startTimer();
            }
            else{
                goToFarPose();
                alignForFar = true;
            }
        }

        if(alignForFar && !follower.isBusy()){
            timer4.startTimer();
            alignForFar = false;
        }

        quitCorrectingAngle();

        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            aligning = false;
            returningHome = false;
            waitingAtHome = false;
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }

        followerstuff();

        // ─── SHOOTING SEQUENCE START LOGIC ────────
        if (depo.reachedTargetHighTolerance()) {
            if (shooting && !waitingToShoot1) {
                waitingToShoot1 = true;
                depoSpinUpTime1 = getRuntime();
                shooting = false;
            }
            if (shooting2 && !waitingToShoot6) {
                waitingToShoot6 = true;
                depoSpinUpTime6 = getRuntime();
                shooting2 = false;
            }
        }

        // Start timer1 after delay
        if (waitingToShoot1 && (getRuntime() - depoSpinUpTime1 >= 0.5) && !timer1.timerIsOn()) {
            timer1.startTimer();
            waitingToShoot1 = false;
        }

        // Start timer6 after delay (this is what runs shootThreeRandom)
        if (waitingToShoot6 && (getRuntime() - depoSpinUpTime6 >= 0.5) && !timer6.timerIsOn()) {
            timer6.startTimer();
            waitingToShoot6 = false;
        }

        // Execute sequences
        if (timer1.timerIsOn()) {
            if (motif.equals("gpp")) {
                if (greenInSlot == 0) shootLRB();
                else if (greenInSlot == 1) shootRBL();
                else shootBLR();
            } else if (motif.equals("pgp")) {
                if (greenInSlot == 0) shootBLR();
                else if (greenInSlot == 1) shootLRB();
                else shootRBL();
            } else {
                if (greenInSlot == 0) shootRBL();
                else if (greenInSlot == 1) shootBLR();
                else shootLRB();
            }
        }

        if (timer6.timerIsOn()) {
            shootThreeRandom();
        }

        // ─── Debug telemetry ─────────────────────────────────────────
        telemetry.addData("Alliance", bluealliance ? "Blue" : "Red");
        telemetry.addData("Ball Count", LL.getBallCount());
        telemetry.addData("Shooter", String.format("%.0f / %d  at target? %b",
                depo.getVelocity(), (int)depo.getTargetVelocity(), depo.reachedTargetHighTolerance()));
        telemetry.addData("shooting2", shooting2);
        telemetry.addData("waitingToShoot6", waitingToShoot6);
        telemetry.addData("timer6 running", timer6.timerIsOn());
        telemetry.addData("Distance to Goal", String.format("%.1f", distanceToGoal));
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Returning Home", returningHome);
        telemetry.addData("Waiting at Home", waitingAtHome);
        if(waitingAtHome) {
            double timeRemaining = 0.5 - (getRuntime() - arrivedAtHomeTime);
            telemetry.addData("Time Until Unlock", String.format("%.2fs", Math.max(0, timeRemaining)));
        }
        Pose cur = follower.getPose();
        telemetry.addData("Position", String.format("(%.1f, %.1f, %.0f°)",
                cur.getX(), cur.getY(), Math.toDegrees(cur.getHeading())));
        telemetry.update();
    }

    // ──────────────────────────────────────────────────────────────
    //  FIXED: Return to home using PathChain
    // ──────────────────────────────────────────────────────────────

    private void returnToHome() {
        if(homePose == null) return;

        Pose cur = follower.getPose();
        Pose targetPose = new Pose(0, 0, 0);

        PathChain homeChain = follower.pathBuilder()
                .addPath(new BezierLine(cur, targetPose))
                .setLinearHeadingInterpolation(cur.getHeading(), 0)
                .build();

        follower.followPath(homeChain);
        returningHome = true;
        telemetry.addLine(">>> RETURNING TO HOME <<<");
    }

    private double getDistanceToHome() {
        Pose cur = follower.getPose();
        double x = 0 - cur.getX();
        double y = 0 - cur.getY();
        return Math.sqrt(x*x + y*y);
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
        return Math.sqrt(x*x + y*y);
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
            timer4.stopTimer();
        }
    }

    private void followerstuff() {
        follower.update();

        if (!follower.isBusy() && aligning2) {
            follower.startTeleopDrive();
            aligning2 = false;
        }

        // Handle return to home completion - wait 0.5s before allowing control
        if (!follower.isBusy() && returningHome && !waitingAtHome) {
            returningHome = false;
            waitingAtHome = true;
            arrivedAtHomeTime = getRuntime();
            telemetry.addLine(">>> ARRIVED AT HOME - WAITING 0.5s <<<");
        }

        // Check if we've waited long enough at home
        if (waitingAtHome && (getRuntime() - arrivedAtHomeTime >= 0.5)) {
            waitingAtHome = false;
            follower.startTeleopDrive();
            telemetry.addLine(">>> HOME POSITION LOCKED - TELEOP ENABLED <<<");
        }

        // Only allow teleop control when not busy with autonomous actions and not waiting
        if (!follower.isBusy() && !aligning && !returningHome && !waitingAtHome) {
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
        pos = LL.sensors.getRight();
        return (pos==1) ? 1 : 2;
    }

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

    private void shootBLR() {
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

    private void shootRBL() {
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

    private void shootThreeRandom() {
        // Shot 1: Left
        if (timer6.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence6 = 1;
        }
        if (timer6.checkAtSeconds(0.3) && shooterSequence6 == 1) {
            LL.allDown();
            shooterSequence6 = 2;
        }

        // Shot 2: Back
        if(shooterSequence6 == 2 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            shooterSequence6 = 3;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }
        if (shooterSequence6 == 3 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.3)) {
            LL.allDown();
            shooterSequence6 = 4;
        }

        // Shot 3: Right
        if(shooterSequence6 == 4 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            shooterSequence6 = 5;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }

        // NEW: Final Down movement gets its own time check
        if (shooterSequence6 == 5 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.3)) {
            LL.allDown();
            shooterSequence6 = 6; // Move to a "closing" state
        }

        // NEW: Stop everything only AFTER the servos have had time to move down
        if (shooterSequence6 == 6 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.5)) {
            depo.setTargetVelocity(0);
            // If your Deposition class has a stop() or setPower(0)
            depo.top.setPower(0);
            depo.bottom.setPower(0);

            timer6.stopTimer();
            shooterSequence6 = 0;
            waitingToShoot6 = false;
        }
    }
}