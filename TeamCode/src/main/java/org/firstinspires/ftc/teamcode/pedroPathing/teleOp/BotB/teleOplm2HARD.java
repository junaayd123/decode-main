package org.firstinspires.ftc.teamcode.pedroPathing.teleOp.BotB;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Lm2 2 hard drivers FIXED ChatGPT+Gemini", group = "TeleOp")
public class teleOplm2HARD extends OpMode {
    // --- Drive / Vision / Pose ---
    private boolean aligning = false;
    private boolean aligning2 = false;
    private boolean alignForFar = false;
    private double distanceToGoal;
    private boolean bluealliance = false;
    private double desiredHeading = 0;
    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(320.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(220.0));
    private final Pose blueFarShootPose = new Pose(80, 25, Math.toRadians(-58));
    private final Pose redFarShootPose  = new Pose(80, 25, Math.toRadians(-115));
    private final Pose redGoal  = new Pose(144, 144, 0);
    private final Pose blueGoal  = new Pose(0, 144,0);
    private final Pose redGoal2  = new Pose(144, 144, 0);
    private final Pose blueGoal2  = new Pose(0, 144,0);
    private final Pose redHP  = new Pose(42, 25, Math.toRadians(180));
    private final Pose blueHP  = new Pose(115, 25,0);

    // --- Vision ---
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public boolean tagDetected;
    public boolean tagInitializing;

    // --- Robot subsystems & telemetry ---
    String motif = "gpp";
    String shotSlot1;
    String shotSlot2;
    String shotSlot3;
    Pose pedroPose, ftcPose;
    int[] ballsInRobot = {0,0,0}; // 0 = empty, 1 = green, 2 = purple
    private DcMotor intake = null;
    private Deposition depo;
    private lift_three LL;
    Servo led;
    double ledRunTime;
    double ledColor;

    // --- Timers & control ---
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    Timer timer5;
    double timeOfSecondShot;
    double ourVelo = 1300;
    boolean shootingTest = false;
    boolean intakeRunning = false;

    // --- Shooter mode and sequences (separated) ---
    // currentMode: 0 = Idle, 1 = 3x automatic, 2 = motif shooter (gpp/pgp/ppg)
    private int currentMode = 0;
    private boolean shooting = false;   // used for starting auto 3x
    private boolean shooting2 = false;  // used for starting motif
    int seq3x = 0;      // state for the 3x shooter
    int seqMotif = 0;   // state for motif shooter

    // --- Misc ---
    int lastShotSlot = -1;
    int shootingHasWorked = -1;
    boolean greenball = false;
    boolean direction = false;
    double speed;
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
        timer5 = new Timer();
        initAprilTag();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        led = hardwareMap.get(Servo.class, "led");

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
        timer3.resetTimer();
        timer4.resetTimer();
        timer5.resetTimer();
        tagInitializing = true;

        // reset shooter states
        seq3x = 0;
        seqMotif = 0;
        currentMode = 0;
        shooting = false;
        shooting2 = false;
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();

        if(g1.psWasPressed()) bluealliance = !bluealliance;

        // Intake toggles
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

        // speed toggle for teleop
        if(g1.cross) speed = 0.3;
        else speed = 1;

        if(g2.dpad_down && !preG2.dpad_down){
            shootingTest = !shootingTest;
        }

        // --- SHOOTING CONTROLS ---
        // Standard 3x shooter (cross)
        if(g2.cross && !preG2.cross){
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                } else {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
                }
                shooting = true;
                currentMode = 1;
            }
        }

        // Motif shooting (square/triangle/circle)
        boolean motifPressed = false;
        if(g2.square && !preG2.square){   // gpp
            motif = "gpp"; motifPressed = true;
        }
        if(g2.triangle && !preG2.triangle){ // pgp
            motif = "pgp"; motifPressed = true;
        }
        if(g2.circle && !preG2.circle){     // ppg
            motif = "ppg"; motifPressed = true;
        }

        if (motifPressed) {
            depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
            LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
            shooting2 = true;
            currentMode = 2;

            // Refresh inventory snapshot from sensors
            ballsInRobot[0] = LL.sensors.getLeft();
            ballsInRobot[1] = LL.sensors.getRight();
            ballsInRobot[2] = LL.sensors.getBack();
        }

        // APRIL TAG localization trigger
        if (g1.triangle && !preG1.triangle) {
            tagInitializing = true;
        }
        if (tagInitializing) {
            updateAprilTagLocalization();
            if (tagDetected && pedroPose != null && !follower.isBusy()) {
                follower.setPose(pedroPose.getPose());
                ledColor = 0.5; // green
                ledRunTime = 0.5;
                timer5.startTimer();
                tagInitializing = false;
            } else {
                led.setPosition(0.388); // yellow
            }
        }

        // drive helpers
        if (g1.square && !preG1.square) {
            goToHumanPlayer();
            direction = !direction;
        }
        if (g1.circle && !preG1.circle && !follower.isBusy()) {
            if(distanceToGoal < 115) {
                faceAllianceGoal();
                timer4.startTimer();
            } else {
                goToFarPose();
                alignForFar = true;
            }
        }
        if (alignForFar && !follower.isBusy()) {
            timer4.startTimer();
            alignForFar = false;
        }
        quitCorrectingAngle();

        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            aligning = false;
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }

        followerstuff();

        // Telemetry basics
        telemetry.addData("Alliance Blue?", bluealliance);
        ledColorTime(ledColor, ledRunTime);
        Pose cur = follower.getPose();
        distanceToGoal = getDistance();
        telemetry.addData("Mode", currentMode == 1 ? "3x Standard" : (currentMode == 2 ? "Motif: " + motif : "Idle"));
        telemetry.addData("Seq 3x", seq3x);
        telemetry.addData("Seq Motif", seqMotif);
        telemetry.addData("first shot", shotSlot1);
        telemetry.addData("second shot", shotSlot2);
        telemetry.addData("third shot", shotSlot3);
        telemetry.addData("actual depo velo", depo.getVelocity());
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("current angle", LL.launchAngleServo.getPosition());
        telemetry.addData("X", cur.getX());
        telemetry.addData("Y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("desired heading", Math.toDegrees(desiredHeading));

        // If depo has reached velocity, start timers for the active shooter mode
        if (depo.reachedTargetHighTolerance()) {
            if (shooting && currentMode == 1) {
                timer1.startTimer();
                shooting = false;
            }
            if (shooting2 && currentMode == 2) {
                timer2.startTimer();
                shooting2 = false;
            }
        }

        // Execute only active shooter
        if (currentMode == 1) {
            shoot3x();
        } else if (currentMode == 2) {
            shootMotifVelo(motif);
        }

        // Manual adjusters
        if(g1.dpad_up && !preG1.dpad_up){
            ourVelo += 25;
        } else if(g1.dpad_down && !preG1.dpad_down){
            ourVelo -= 25;
        }
        if(g1.dpad_left && !preG1.dpad_left){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition() - 0.03);
        } else if(g1.dpad_right && !preG1.dpad_right){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition() + 0.03);
        }

        telemetry.update();
    }

    // --- helper methods & subsystems below ---

    private void ledColorTime(double color, double sec) {
        if (timer5.checkAtSeconds(0)) {
            led.setPosition(color);
        }
        if (timer5.checkAtSeconds(sec)) {
            led.setPosition(0);
            timer5.stopTimer();
        }
    }

    private int veloBasedOnDistance(double dist) {
        if (dist < 60) return 1125;
        else if (dist < 70) return 1150;
        else if (dist < 75) return 1175;
        else if (dist < 80) return 1200;
        else if (dist < 87) return 1225;
        else if (dist < 110) return 1300;
        else if (dist > 115 && dist < 150) return 1575;
        else return 0;
    }

    private double angleBasedOnDistance(double dist) {
        if (dist < 70) return 0.06;
        else if (dist < 87) return 0.09;
        else if (dist < 110) return 0.12;
        else if (dist > 115 && dist < 150) return 0.18;
        else return 0.06;
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

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
                telemetry.addLine("Warning: Webcam not found");
            }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double getDistance() {
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal2 : redGoal2;
        double x = target.getX() - cur.getX();
        double y = target.getY() - cur.getY();
        return Math.sqrt(x * x + y * y);
    }

    private void updateAprilTagLocalization() {
        if (aprilTag == null) return;
        List<AprilTagDetection> dets = aprilTag.getDetections();
        tagDetected = false;
        for (AprilTagDetection d : dets) {
            if (d.metadata == null) continue;
            if (d.metadata.name.contains("Obelisk")) continue;
            tagDetected = true;
            double xIn = d.robotPose.getPosition().x;
            double yIn = d.robotPose.getPosition().y;
            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            pedroPose = new Pose(ftcPose.getY() + 72, -ftcPose.getX() + 72, ftcPose.getHeading());
            break;
        }
    }

    private void goToAllianceShootingPose() {
        Pose target = bluealliance ? blueNearShootPose : redNearShootPose;
        if (pedroPose == null) return;
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(pedroPose, target))
                .setLinearHeadingInterpolation(pedroPose.getHeading(), target.getHeading())
                .build();
        follower.followPath(chain);
        aligning2 = true;
    }

    private void goToFarPose() {
        Pose target = bluealliance ? blueFarShootPose : redFarShootPose;
        Pose cur = follower.getPose();
        if (pedroPose == null) return;
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();
        follower.followPath(chain);
    }

    private void goToHumanPlayer() {
        Pose target = bluealliance ? blueHP : redHP;
        if (pedroPose == null) return;
        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(pedroPose, target))
                .setLinearHeadingInterpolation(pedroPose.getHeading(), target.getHeading())
                .build();
        follower.followPath(chain);
        aligning2 = true;
    }

    private void faceAllianceGoal() {
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal : redGoal;
        if (pedroPose == null) return;
        double rawAngle = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());
        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
        desiredHeading = flippedAngle;
        follower.turnTo(flippedAngle);
        aligning = true;
    }

    private void quitCorrectingAngle() {
        if (timer4.checkAtSeconds(1)) {
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
            if (direction) {
                follower.setTeleOpDrive(gamepad1.left_stick_y * speed, (gamepad1.right_trigger - gamepad1.left_trigger) * speed, -gamepad1.right_stick_x * speed, true);
            } else {
                follower.setTeleOpDrive(-gamepad1.left_stick_y * speed, (gamepad1.left_trigger - gamepad1.right_trigger) * speed, -gamepad1.right_stick_x * speed, true);
            }
        }
    }

    private boolean turnIsBasicallyDone() {
        Pose cur = follower.getPose();
        double error = Math.abs(desiredHeading - cur.getHeading());
        error = Math.abs((error + Math.PI) % (2 * Math.PI) - Math.PI);
        return error < Math.toRadians(2);
    }

    // =========================
    // 3x Shooter (auto) using seq3x
    // =========================
    private void shoot3x() {
        if (timer1.checkAtSeconds(0)) {
            LL.leftUp();
            seq3x = 1;
        }

        if (timer1.checkAtSeconds(0.4) && seq3x == 1) {
            LL.allDown();
            seq3x = 2;
        }
        if (seq3x == 2 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            seq3x = 3;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && seq3x == 3) {
            LL.allDown();
            seq3x = 4;
        }
        if (seq3x == 4 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            seq3x = 5;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }

        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && seq3x == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            seq3x = 0;
            currentMode = 0; // return to idle
        }
    }

    // =========================
    // Motif shooter using seqMotif (Gemini logic + consumption)
    // =========================
    private void shootMotifVelo(String seq) {
        int c1 = 0, c2 = 0, c3 = 0;
        if (seq.equals("gpp"))      { c1 = 1; c2 = 2; c3 = 2; }
        else if (seq.equals("pgp")) { c1 = 2; c2 = 1; c3 = 2; }
        else if (seq.equals("ppg")) { c1 = 2; c2 = 2; c3 = 1; }

        // Shot 1
        if (timer2.checkAtSeconds(0) && seqMotif == 0) {
            fireNextBallOfColor(c1, 1);
            seqMotif = 1;
        }

        // Lower after shot 1
        if (timer2.checkAtSeconds(0.4) && seqMotif == 1) {
            LL.allDown();
            seqMotif = 2;
        }

        // Shot 2 (wait for velocity)
        if (seqMotif == 2 && depo.reachedTargetHighTolerance()) {
            fireNextBallOfColor(c2, 2);
            seqMotif = 3;
            timeOfSecondShot = timer2.timer.seconds() - timer2.curtime;
        }

        // Lower after shot 2
        if (timer2.checkAtSeconds(0.4 + timeOfSecondShot) && seqMotif == 3) {
            LL.allDown();
            seqMotif = 4;
        }

        // Shot 3 (wait for velocity)
        if (seqMotif == 4 && depo.reachedTargetHighTolerance()) {
            fireNextBallOfColor(c3, 3);
            seqMotif = 5;
            timeOfSecondShot = timer2.timer.seconds() - timer2.curtime;
        }

        // Finish cycle
        if (timer2.checkAtSeconds(0.4 + timeOfSecondShot) && seqMotif == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            seqMotif = 0;
            currentMode = 0; // return to idle
        }
    }

    /**
     * Finds the next available ball that matches colorCode (1=green,2=purple).
     * If found, writes telemetry about it (color + slot), then commands the lift and
     * marks the slot empty. If none found, falls back to firing the first available ball.
     *
     * Returns true if something was fired, false if nothing to fire.
     */
    private boolean fireNextBallOfColor(int colorCode, int shotNum) {
        // prefer the requested color
        int slotToShoot = -1;

        if (ballsInRobot[0] == colorCode) slotToShoot = 0;
        else if (ballsInRobot[1] == colorCode) slotToShoot = 1;
        else if (ballsInRobot[2] == colorCode) slotToShoot = 2;

        // If none of requested color found, fallback to first available slot
        if (slotToShoot == -1) {
            if (ballsInRobot[0] != 0) slotToShoot = 0;
            else if (ballsInRobot[1] != 0) slotToShoot = 1;
            else if (ballsInRobot[2] != 0) slotToShoot = 2;
        }

        if (slotToShoot == -1) {
            // nothing to shoot
            return false;
        }

        // Build telemetry string (determine color BEFORE we zero the slot)
        String colorName = (ballsInRobot[slotToShoot] == 1) ? "Green" : (ballsInRobot[slotToShoot] == 2) ? "Purple" : "Unknown";
        String slotName = (slotToShoot == 0) ? "left" : (slotToShoot == 1) ? "right" : "back";
        String message = colorName + " from " + slotName;

        // set telemetry slot text
        if (shotNum == 1) shotSlot1 = message;
        else if (shotNum == 2) shotSlot2 = message;
        else if (shotNum == 3) shotSlot3 = message;

        // Command the lift for this slot, then mark it empty
        liftSlot(slotToShoot);
        ballsInRobot[slotToShoot] = 0;
        return true;
    }

    private void liftSlot(int slot) {
        if (slot == 0) LL.leftUp();
        else if (slot == 1) LL.rightUp();
        else if (slot == 2) LL.backUp();
    }

    private void checkShot() {
        if (shootingHasWorked == -1) {
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            LL.allDown();
            seqMotif = 0;
            currentMode = 0;
        }
    }

    private void shootoneColored() {
        if (timer2.checkAtSeconds(0)) {
            if (greenball) LL.lift_green();
            else LL.lift_purple();
        }
        if (timer2.checkAtSeconds(0.3)) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
        }
    }
}
