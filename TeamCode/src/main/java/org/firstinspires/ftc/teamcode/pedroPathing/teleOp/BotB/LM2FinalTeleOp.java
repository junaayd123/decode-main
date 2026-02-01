package org.firstinspires.ftc.teamcode.pedroPathing.teleOp.BotB;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "LM2 Final teleOp", group = "TeleOp")
public class LM2FinalTeleOp extends OpMode {
    // Movement Flags
    private boolean aligning = false;
    private boolean movingToTarget = false;
    private double arrivedAtTargetTime = 0;
    private boolean waitingAtTarget = false;
    private boolean bluealliance = false;
    private boolean posFound = false;

    // Vision Targets
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            -3, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    // TUNE: Distance from AprilTag when positioning (in inches)

    private DcMotor intake = null;
    private Deposition depo;
    private lift_three LL;
    private Follower follower;

    // Control Variables
    private boolean intakeRunning;
    private boolean direction = false;
    private double speed;
    private int loopCounter = 0;

    // Timers
    Timer timer1, timer3, timer4, timer6;

    // Shooting Variables
    boolean shooting2 = false;
    boolean waitingToShoot1 = false;
    boolean waitingToShoot6 = false;
    double depoSpinUpTime1 = 0;
    double depoSpinUpTime6 = 0;
    int shooterSequence1, shooterSequence6;
    double timeOfSecondShot6;
    double timeOfThirdShot6;  // NEW: separate variable for third shot timing

    Gamepad g1 = new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2 = new Gamepad();
    Gamepad preG2 = new Gamepad();

    private final Pose startPose = new Pose(0,0,0);
    private final Pose redShootPose = new Pose(84, 84, 45);
    // other shooting stuff
    private double headingToGoal = 0; //MAKE THIS IN PEDRO RIGHT-HAND COORDINATES, NOT FTC COORDS

    private String motif = "ppg";

    @Override
    public void init() {
        LL = new lift_three(hardwareMap);
        depo = new Deposition(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        timer1 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer6 = new Timer();

        // Init Vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
        depo.setTargetVelocity(0);
        depo.kF = 0.00048;
        LL.allDown();
        LL.set_angle_min();
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

        // ========= 1. SQUARE: TAKE SNAPSHOT & RELOCALIZE =========
        if (g1.square && !preG1.square) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            posFound = false;
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    double ftcX = detection.robotPose.getPosition().x;
                    double ftcY = detection.robotPose.getPosition().x;
                    double ftcHeading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // Get other detection info
                    double range = detection.ftcPose.range;
                    double bearing = Math.toRadians(detection.ftcPose.bearing);
                    double yaw = Math.toRadians(detection.ftcPose.yaw);

                    Pose nextPose = new Pose(ftcY+72, -ftcX+72,ftcHeading);

                    // Get current robot heading
                    double robotHeading = follower.getPose().getHeading();

                    // Update robot pose
                    follower.setPose(nextPose);
                    posFound = true;
                    telemetry.addLine(">>> TARGET LOCKED <<<");
                    //telemetry.addData("Current Distance", String.format("%.1f inches", currentDistance));
                    //telemetry.addData("Target Distance", String.format("%.1f inches", targetDistanceFromTag));
                } else if (detection.metadata.name.contains("Obelisk")) {
                    if (detection.id == 21) {
                        motif = "gpp";
                    }
                    if (detection.id == 22) {
                        motif = "pgp";
                    }
                    if (detection.id == 23) {
                        motif = "ppg";
                    }
                }
            }
            if (!posFound) telemetry.addLine("!!! NO TAG DETECTED !!!");
        }

        // ========= 2. TRIANGLE: DRIVE TO SNAPSHOT POSE =========
        /*if (g1.triangle && !preG1.triangle && visionTargetPose != null) {
            if(movingToTarget || waitingAtTarget) {
                // Cancel movement to target
                follower.breakFollowing();
                follower.startTeleopDrive();
                movingToTarget = false;
                waitingAtTarget = false;
                telemetry.addLine(">>> MOVEMENT TO TARGET CANCELLED <<<");
            } else if(!follower.isBusy()) {
                // Start movement to target
                moveToVisionTarget();
            }
        }*/

        // ========= FORCE CANCEL WITH DPAD DOWN =========
        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            movingToTarget = false;
            waitingAtTarget = false;
            aligning = false;
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }

        // ========= INTAKE & DRIVE CONTROL =========
        if (g2.right_bumper && !preG2.right_bumper) {
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

        // ========= SHOOTING LOGIC =========
        // Prevent shooting triggers from being pressed while already shooting
        if(g2.triangle && !preG2.triangle && !timer6.timerIsOn() && !waitingToShoot6) {
            depo.setTargetVelocity(1350);
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

        // ─── SHOOTING SEQUENCE START LOGIC ────────
        if (depo.reachedTargetHighTolerance()) {
            if (shooting2 && !waitingToShoot6) {
                waitingToShoot6 = true;
                depoSpinUpTime6 = getRuntime();
                shooting2 = false;
            }
        }

        // Start timer6 after delay (this is what runs shootThreeRandom)
        if (waitingToShoot6 && (getRuntime() - depoSpinUpTime6 >= 0.5) && !timer6.timerIsOn()) {
            timer6.startTimer();
            waitingToShoot6 = false;
        }

        // Execute sequences
        if (timer6.timerIsOn()) {
            shootThreeRandom();
        }

        followerstuff();

        // ========= TELEMETRY =========
        //telemetry.addData("Target Locked", visionTargetPose != null);
        //telemetry.addData("Target Distance Setting", targetDistanceFromTag + " inches");
        telemetry.addData("Moving to Target", movingToTarget);
        telemetry.addData("Waiting at Target", waitingAtTarget);
        telemetry.addData("Ball Count", LL.getBallCount());
        telemetry.addData("Shooter", String.format("%.0f / %d  at target? %b",
                depo.getVelocity(), (int)depo.getTargetVelocity(), depo.reachedTargetHighTolerance()));
        telemetry.addData("shooting2", shooting2);
        telemetry.addData("waitingToShoot6", waitingToShoot6);
        telemetry.addData("timer6 running", timer6.timerIsOn());
        telemetry.addData("shooterSequence6", shooterSequence6);
        if(waitingAtTarget) {
            double timeRemaining = 0.5 - (getRuntime() - arrivedAtTargetTime);
            telemetry.addData("Time Until Unlock", String.format("%.2fs", Math.max(0, timeRemaining)));
        }
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // ========= MOVEMENT TO VISION TARGET =========
    /*private void moveToVisionTarget() {

        Pose cur = follower.getPose();

        PathChain targetChain = follower.pathBuilder()
                .addPath(new BezierLine(cur, visionTargetPose))
                .setLinearHeadingInterpolation(cur.getHeading(), visionTargetPose.getHeading())
                .build();

        follower.followPath(targetChain);
        movingToTarget = true;
        telemetry.addLine(">>> MOVING TO TARGET <<<");
    }*/

    /*private void shootThreeRandom() {
        // Shot 1: Left
        if (timer6.checkAtSeconds(0) && shooterSequence6 == 0) {
            LL.leftUp();
            shooterSequence6 = 1;
            return; // Exit after this action
        }
        if (timer6.checkAtSeconds(0.25) && shooterSequence6 == 1) {
            LL.allDown();
            shooterSequence6 = 2;
            return;
        }

        // Shot 2: Back
        // THE COMMENTED OUT LINE IS FOR CHECKING IF THE DEPO HAS REACHED THE HIGH TOLERANCE!
        //if(shooterSequence6 == 2 && depo.reachedTargetHighTolerance()) {
        if(shooterSequence6 == 2) {
            LL.backUp();
            shooterSequence6 = 3;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
            return;
        }
        if (shooterSequence6 == 3 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.25)) {
            LL.allDown();
            shooterSequence6 = 4;
            return;
        }

        // Shot 3: Right
        //again another commented out one for checking flywheel speed
        //if(shooterSequence6 == 4 && depo.reachedTargetHighTolerance()) {
        if(shooterSequence6 == 4) {
            LL.rightUp();
            shooterSequence6 = 5;
            timeOfThirdShot6 = timer6.timer.seconds() - timer6.curtime;
            return;
        }

        if (shooterSequence6 == 5 && timer6.checkAtSeconds(timeOfThirdShot6 + 0.25)) {
            LL.allDown();
            shooterSequence6 = 6;
            return;
        }

        if (shooterSequence6 == 6 && timer6.checkAtSeconds(timeOfThirdShot6 + 0.5)) {
            depo.setTargetVelocity(0);
            depo.top.setPower(0);
            depo.bottom.setPower(0);
            timer6.stopTimer();
            shooterSequence6 = 0;
            waitingToShoot6 = false;
        }
    }*/
    private void shootThreeRandom() {
        // Shot 1: Left
        if (timer6.checkAtSeconds(0) && shooterSequence6 == 0) {
            LL.leftUp();
            shooterSequence6 = 1;
            return;
        }
        if (timer6.checkAtSeconds(0.25) && shooterSequence6 == 1) {
            LL.allDown();
            LL.backUp();  // Start shot 2 immediately
            shooterSequence6 = 2;
            return;
        }

        // Shot 2: Back - FIXED TIME
        if (timer6.checkAtSeconds(0.5) && shooterSequence6 == 2) {
            LL.allDown();
            LL.rightUp();  // Start shot 3 immediately
            shooterSequence6 = 3;
            return;
        }

        // Shot 3: Right - FIXED TIME
        if (timer6.checkAtSeconds(0.75) && shooterSequence6 == 3) {
            LL.allDown();
            shooterSequence6 = 4;
            return;
        }

        // sets everything to 0
        // this is where the new function gets used
        if (timer6.checkAtSecondsTolerance(1.0, 0.1) && shooterSequence6 == 4) {
            depo.setTargetVelocity(0);
            depo.top.setPower(0);
            depo.bottom.setPower(0);
            timer6.stopTimer();
            shooterSequence6 = 0;
            waitingToShoot6 = false;
        }
    }

    private void reverseIntake() {
        if (timer3.checkAtSeconds(0)) intake.setPower(1);
        if (timer3.checkAtSeconds(0.5)) {
            intake.setPower(0);
            timer3.stopTimer();
        }
    }

    private void followerstuff() {
        follower.update();

        // Handle movement to target completion - wait 0.5s before allowing control
        if (!follower.isBusy() && movingToTarget && !waitingAtTarget) {
            movingToTarget = false;
            waitingAtTarget = true;
            arrivedAtTargetTime = getRuntime();
            telemetry.addLine(">>> ARRIVED AT TARGET - WAITING 0.5s <<<");
        }

        // Check if we've waited long enough at target
        if (waitingAtTarget && (getRuntime() - arrivedAtTargetTime >= 0.5)) {
            waitingAtTarget = false;
            follower.startTeleopDrive();
            telemetry.addLine(">>> TARGET POSITION LOCKED - TELEOP ENABLED <<<");
        }

        // Only allow teleop control when not busy with autonomous actions and not waiting
        if (!follower.isBusy() && !movingToTarget && !waitingAtTarget) {
            double ly = direction ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
            double lx = direction ? (gamepad1.right_trigger - gamepad1.left_trigger) : (gamepad1.left_trigger - gamepad1.right_trigger);
            follower.setTeleOpDrive(ly * speed, lx * speed, -gamepad1.right_stick_x * speed, true);
        }
    }
}