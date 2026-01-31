// ===================== BLUE AUTO (FAR SHOT ONLY) =====================
// All later sequences (gate cycles, pickups, extra shots, parking) are commented out.

package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Obsolete;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Farblue testing ", group = "Pedro")
public class farbluetesting extends OpMode {

    // =========== SUBSYSTEMS ===========
    private Follower follower;
    private Deposition_C depo;
    private TurretLimelight turret;
    private lifters LL;
    private ColorSensors sensors;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;

    // ========== VISION ==========
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    // ========== TIMERS ==========
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int shooterSequence;
    private int greenInSlot;
    private String motif = "empty";
    private int gateHitCount = 0;
    private int shotCycleCount = 0;
    private boolean intakeRunning = false;

    // ======== CONSTANTS ==========
    private static double SHOOT_INTERVAL = 0.335;
    private static final double SETTLE_TIME = 0.3;

    // ========== POSES ==========
    private final Pose startPose = new Pose(13.5, -7, Math.toRadians(0));
    private final Pose farshotpose = new Pose(12, -17, Math.toRadians(0));

    // ========== PATHS ==========
    private PathChain goToFarShot;

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();

        depo = new Deposition_C(hardwareMap);
        LL = new lifters(hardwareMap);
        sensors = new ColorSensors(hardwareMap);
        turret = new TurretLimelight(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        d1 = hardwareMap.get(DcMotor.class, "depo");
        d2 = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        LL.allDown();
        LL.set_angle_min();
        stopShooter();

        turret.resetTurretEncoder();
        turret.setDegreesTarget(68.5);

        initAprilTag();

        telemetry.addLine("Far Shot Only Auto Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();
        detectMotifFromAprilTags();
        telemetry.addData("Motif", motif);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        shotCycleCount = 0;
        setPathState(0);
        setActionState(0);
    }

    @Override
    public void loop() {
        follower.update();
        turret.toTargetInDegrees();

        autonomousPathUpdate();
        autonomousActionUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.update();
    }

    // ========== APRILTAG ==========
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(648.975, 648.975, 304.535, 221.714)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void detectMotifFromAprilTags() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                if (detection.id == 21) motif = "gpp";
                if (detection.id == 22) motif = "pgp";
                if (detection.id == 23) motif = "ppg";
            }
        }
    }

    // ========== PATH STATE MACHINE ==========
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Spin up shooter
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloblueauto);
                SHOOT_INTERVAL = 0.375;
                setPathState(1);
                break;

            case 1: // Wait for shooter to reach speed
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2: // Drive to far shot pose
                buildFarShotPath();
                follower.followPath(goToFarShot, true);
                setPathState(3);
                break;

            case 3: // Wait until at far shot pose
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4: // Settle before shooting
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(5);
                }
                break;

            case 5: // Wait for shooting to finish
                if (actionState == 0) {
                    setPathState(-1); // END AUTO
                }
                break;

            // ================================
            // EVERYTHING BELOW THIS IS COMMENTED OUT
            // ================================

//            case 6:
//            case 7:
//            case 8:
//            case 9:
//            case 10:
//            case 11:
//            case 12:
//            case 13:
//            case 14:
//            case 15:
//            case 16:
//            case 17:
//            case 18:
//                break;
        }
    }

    private void buildFarShotPath() {
        Pose cur = follower.getPose();
        goToFarShot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, farshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), farshotpose.getHeading())
                .build();
    }

    // ========== ACTION STATE MACHINE ==========
    public void autonomousActionUpdate() {
        switch (actionState) {

            case 0:
                break;

            case 1: // Start shooting
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloblueauto);

                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 2: // Wait for shooter speed
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                }
                break;

            case 3: // Shoot 3 rings
                depo.updatePID();
                shootThreeRandom();

                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 3) {
                    LL.allDown();
                    depo.setTargetVelocity(0);
                    stopShooter();
                    setActionState(0);
                }
                break;
        }
    }

    // ========== SHOOTING HELPERS ==========
    private void shootThreeRandom() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t < SHOOT_INTERVAL - 0.05) LL.leftUp();
        else if (t < SHOOT_INTERVAL) LL.allDown();
        else if (t < SHOOT_INTERVAL * 2 - 0.05) LL.rightUp();
        else if (t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t < SHOOT_INTERVAL * 3) LL.backUp();
    }

    private int getGreenPos() {
        int pos = LL.sensors.getLeft();
        if (pos == 1) return 0;
        pos = LL.sensors.getRight();
        if (pos == 1) return 1;
        return 2;
    }

    private void stopShooter() {
        if (d1 != null) d1.setPower(0);
        if (d2 != null) d2.setPower(0);
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
    }

    @Override
    public void stop() {
        stopShooter();
        if (intake != null) intake.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }
}

