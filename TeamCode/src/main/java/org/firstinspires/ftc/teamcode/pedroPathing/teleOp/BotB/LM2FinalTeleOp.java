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

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "LM2 Final teleOp", group = "TeleOp")
public class LM2FinalTeleOp extends OpMode {
    // Movement Flags
    private boolean aligning = false;
    private boolean returningHome = false;
    private double arrivedAtHomeTime = 0;
    private boolean waitingAtHome = false;
    private boolean bluealliance = false;

    // Vision Targets
    private Pose visionTargetPose = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // TUNE: Camera lens distance from center of robot
    private double cameraOffsetRadius = 4.0;
    private double cameraOffsetAngle = 0;

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
    double timeOfSecondShot1, timeOfSecondShot6;

    Gamepad g1 = new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2 = new Gamepad();
    Gamepad preG2 = new Gamepad();

    private final Pose startPose = new Pose(0,0,0);

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
            boolean found = false;
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    // Update Pedro's internal location based on camera
                    double range = detection.ftcPose.range;
                    double bearing = Math.toRadians(detection.ftcPose.bearing);
                    double robotHeading = follower.getPose().getHeading();

                    double tagX = detection.metadata.fieldPosition.get(0);
                    double tagY = detection.metadata.fieldPosition.get(1);

                    double theta = robotHeading - bearing;
                    double robX = tagX - (range * Math.cos(theta));
                    double robY = tagY - (range * Math.sin(theta));

                    follower.setPose(new Pose(robX, robY, robotHeading));

                    // Set target: 100 inches away, 90 degrees orientation
                    // Assuming X-axis movement for "away"
                    visionTargetPose = new Pose(tagX - 100.0, tagY, Math.toRadians(90));
                    found = true;
                    break;
                }
            }
            if (!found) telemetry.addLine("!!! NO TAG DETECTED !!!");
        }

        // ========= 2. TRIANGLE: DRIVE TO SNAPSHOT POSE =========
        if (g1.triangle && !preG1.triangle && visionTargetPose != null) {
            PathChain alignPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), visionTargetPose))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), visionTargetPose.getHeading())
                    .build();
            follower.followPath(alignPath);
        }

        // ========= INTAKE & DRIVE CONTROL =========
        if (g2.right_bumper && !preG2.right_bumper) {
            if (intake.getPower() < -0.5) intake.setPower(0);
            else intake.setPower(-1);
        }
        reverseIntake();
        speed = g1.right_bumper ? 0.3 : 1;

        // ========= SHOOTING LOGIC =========
        boolean isShooting = timer1.timerIsOn() || timer6.timerIsOn() || waitingToShoot1 || waitingToShoot6;

        if(g2.triangle && !preG2.triangle && !isShooting) {
            depo.setTargetVelocity(1300);
            LL.set_angle_close();
            shooting2 = true;
        }

        if (depo.reachedTargetHighTolerance()) {
            if (shooting2 && !waitingToShoot6) {
                waitingToShoot6 = true;
                depoSpinUpTime6 = getRuntime();
                shooting2 = false;
            }
        }

        if (waitingToShoot6 && (getRuntime() - depoSpinUpTime6 >= 0.5) && !timer6.timerIsOn()) {
            timer6.startTimer();
            waitingToShoot6 = false;
        }

        if (timer6.timerIsOn()) shootThreeRandom();

        followerstuff();

        telemetry.addData("Target Locked", visionTargetPose != null);
        telemetry.update();
    }

    private void shootThreeRandom() {

        // Shot 1
        if (timer6.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence6 = 1;
        }
        if (timer6.checkAtSeconds(0.3) && shooterSequence6 == 1) {
            LL.allDown();
            shooterSequence6 = 2;
        }
        // Shot 2
        if(shooterSequence6 == 2 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            shooterSequence6 = 3;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }
        if (shooterSequence6 == 3 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.3)) {
            LL.allDown();
            shooterSequence6 = 4;
        }
        // Shot 3
        if(shooterSequence6 == 4 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            shooterSequence6 = 5;
            timeOfSecondShot6 = timer6.timer.seconds() - timer6.curtime;
        }
        // FINAL: Bring all down and STOP DEPO
        if (shooterSequence6 == 5 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.3)) {
            LL.allDown();
            shooterSequence6 = 6;
        }
        if (shooterSequence6 == 6 && timer6.checkAtSeconds(timeOfSecondShot6 + 0.5)) {
            depo.stop(); // Stops motors and target velocity
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
        if (!follower.isBusy()) {
            double ly = direction ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
            double lx = direction ? (gamepad1.right_trigger - gamepad1.left_trigger) : (gamepad1.left_trigger - gamepad1.right_trigger);
            follower.setTeleOpDrive(ly * speed, lx * speed, -gamepad1.right_stick_x * speed, true);
        }
    }
}