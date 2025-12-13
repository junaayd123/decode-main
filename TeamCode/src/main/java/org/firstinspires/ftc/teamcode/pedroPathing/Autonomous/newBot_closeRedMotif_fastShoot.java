package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "very bad close red", group = "Pedro")
public class newBot_closeRedMotif_fastShoot extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    String motif = "empty";
    String motifInit = "empty";

    private Deposition depo;
    private lift_three LL;
    private DcMotor intake;

    private DcMotor d1, d2;

    private Follower follower;

    private Timer timer1, timer2;
    private int sequence = 0;

    double timeOfSecondShot;
    int shooterSequence;
    int greenInSlot;

    private static final double SECOND_HOP_IN = 20.0;

    // ---------------- POSES (UNCHANGED) ----------------

    private final Pose startPose = new Pose(107.313, -29.687, Math.toRadians(-135));
    private final Pose nearshotpose = new Pose(90, -8.5, Math.toRadians(-220));
    private final Pose firstpickupPose = new Pose(66.5, -6.5, Math.toRadians(-90));
    private final Pose secondpickupPose = new Pose(43.25, -8, Math.toRadians(-90));
    private final Pose midpoint1 = new Pose(43.25, -2, Math.toRadians(90));
    private final Pose thirdpickupPose = new Pose(20, -9.5, Math.toRadians(-90));
    private final Pose homePose = new Pose(0, 0, Math.toRadians(-25));
    private final Pose infront_of_lever = new Pose(61.5, -37.5, Math.toRadians(180));

    @Override
    public void runOpMode() {

        depo = new Deposition(hardwareMap);
        LL = new lift_three(hardwareMap);
        timer1 = new Timer();
        timer2 = new Timer();

        intake = hardwareMap.get(DcMotor.class, "intake");
        d1 = hardwareMap.get(DcMotor.class, "depo");
        d2 = hardwareMap.get(DcMotor.class, "depo1");

        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        initAprilTag();

        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();

        while (!isStarted() && !isStopRequested()) {
            if (motifInit.equals("empty")) InitialFindMotif();

            if (motifInit.equals("ppg")) {
                motif = "gpp";
                motifInit = "empty";
            } else if (motifInit.equals("pgp")) {
                motif = "ppg";
                motifInit = "empty";
            } else if (motifInit.equals("gpp")) {
                motif = "pgp";
                motifInit = "empty";
            }

            telemetry.addData("Motif", motif);
            telemetry.update();
            idle();
        }

        waitForStart();
        if (isStopRequested()) return;

        go_back();
        pauseBeforeShooting(0.4);
        three_close_shots();

        first_line_pickup();
        reset();
        go_close();
        three_close_shots();

        second_line_pickup();
        reset();
        go_close();
        three_close_shots();

        third_line_pickup();
        reset();
        go_close();
        three_close_shots();

        go_infront();
    }

    // ---------------- SHOOTING ----------------

    private void startCloseShot() {
        sequence = 4;
        depo.setTargetVelocity(depo.closeVelo_New);
    }

    private void three_close_shots() {
        LL.set_angle_close();
        startCloseShot();

        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();

            if (depo.reachedTarget()) {
                if (sequence == 3 || sequence == 4) {
                    greenInSlot = getGreenPos();
                    timer1.startTimer();
                    sequence = 0;
                }
            }

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

            follower.update();
        }
    }

    private boolean isFarShotCycleDone() {
        return (sequence == 0 && timer1.timerIsOff());
    }

    private int getGreenPos() {
        if (LL.sensors.getLeft() == 1) return 0;
        if (LL.sensors.getRight() == 1) return 1;
        return 2;
    }

    private void shootLRB() {
        if (timer1.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence = 1;
        }
        if (timer1.checkAtSeconds(0.4) && shooterSequence == 1) {
            LL.allDown();
            shooterSequence = 2;
        }
        if (shooterSequence == 2 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            shooterSequence = 3;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 3) {
            LL.allDown();
            shooterSequence = 4;
        }
        if (shooterSequence == 4 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            shooterSequence = 5;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }

    private void shootBLR() {
        if (timer1.checkAtSeconds(0)) {
            LL.backUp();
            shooterSequence = 1;
        }
        if (timer1.checkAtSeconds(0.4) && shooterSequence == 1) {
            LL.allDown();
            shooterSequence = 2;
        }
        if (shooterSequence == 2 && depo.reachedTargetHighTolerance()) {
            LL.leftUp();
            shooterSequence = 3;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 3) {
            LL.allDown();
            shooterSequence = 4;
        }
        if (shooterSequence == 4 && depo.reachedTargetHighTolerance()) {
            LL.rightUp();
            shooterSequence = 5;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }

    private void shootRBL() {
        if (timer1.checkAtSeconds(0)) {
            LL.rightUp();
            shooterSequence = 1;
        }
        if (timer1.checkAtSeconds(0.4) && shooterSequence == 1) {
            LL.allDown();
            shooterSequence = 2;
        }
        if (shooterSequence == 2 && depo.reachedTargetHighTolerance()) {
            LL.backUp();
            shooterSequence = 3;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 3) {
            LL.allDown();
            shooterSequence = 4;
        }
        if (shooterSequence == 4 && depo.reachedTargetHighTolerance()) {
            LL.leftUp();
            shooterSequence = 5;
            timeOfSecondShot = timer1.timer.seconds() - timer1.curtime;
        }
        if (timer1.checkAtSeconds(0.4 + timeOfSecondShot) && shooterSequence == 5) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }

    // ---------------- APRILTAG ----------------

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void InitialFindMotif() {
        try {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null &&
                        detection.metadata.name.contains("Obelisk")) {
                    motifInit = (detection.id == 21) ? "gpp"
                            : (detection.id == 22) ? "pgp" : "ppg";
                }
            }
        } catch (Exception ignored) {}
    }

    // ---------------- MOVEMENT ----------------

    private void pauseBeforeShooting(double seconds) {
        Timer t = new Timer();
        t.startTimer();
        while (opModeIsActive() && !t.checkAtSeconds(seconds)) idle();
    }

    private void reset() {
        depo.setPowerBoth(0);
    }

    private void go_back() {
        Pose cur = follower.getPose();
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
    }

    private void go_close() {
        Pose cur = follower.getPose();
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
        intake.setPower(0);
    }

    private void go_infront() {
        Pose cur = follower.getPose();
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), homePose.getHeading())
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
    }

    private void first_line_pickup() {
        intake.setPower(-1);
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierLine(nearshotpose, firstpickupPose)))
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
    }

    private void second_line_pickup() {
        intake.setPower(-1);
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(nearshotpose, secondpickupPose)))
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
    }

    private void third_line_pickup() {
        intake.setPower(-1);
        Pose cur = follower.getPose();
        PathChain p = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, thirdpickupPose)))
                .build();
        follower.followPath(p, true);
        while (opModeIsActive() && follower.isBusy()) follower.update();
    }
}
