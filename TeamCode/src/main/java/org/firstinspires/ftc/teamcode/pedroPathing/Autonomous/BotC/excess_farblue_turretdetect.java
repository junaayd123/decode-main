package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.BotC;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_Intensity;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/**
 * excess_farblue_turretdetect
 *
 * Turret-detect variant — after excess shot, turns turret to scan for balls
 * WITHOUT driving to a detection midpoint first. Detect in place, then branch.
 * Sequence: Preload → Third line pickup → Shot → Excess → Shot → Detect → Branch
 */
@Autonomous(name = "excess_farblue_turretdetect", group = "Pedro")
public class excess_farblue_turretdetect extends OpMode {

    // =========== SUBSYSTEMS ===========
    private Follower follower;
    private Deposition_C depo;
    private TurretLimelight turret;
    private lifters LL;
    private ColorSensors_Intensity intensitySensors;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;
    private Servo camTilt = null;

    // ========== VISION ==========
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private BallCoverageProcessor ballCoverage;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    // ========== TIMERS ==========
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private Timer excessPathTimeoutTimer;

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int greenInSlot;
    private int excessPickupCount = 0;
    private int shotCycleCount = 0;
    private int scanCycleCount = 0;
    private int ballCount = 3;
    private double savedFirstRoiCombined = 0;

    // ========== MOTIF ==========
    private String motif = "ppg";
    private String detectedMotif = "";
    private boolean motifLocked = false;
    private int motifDetectionCount = 0;
    private static final int MOTIF_DETECTION_THRESHOLD = 3;

    // ========== CONSTANTS ==========
    private static double SHOOT_INTERVAL = 0.375;
    private static final double SETTLE_TIME = 0.15;
    private static final double SPIT_DURATION_SEC = 0.25;
    private static final double EXCESS_WAIT_FIRST_POSITION = 1;
    private static final double EXCESS_WAIT_SECOND_POSITION = 1;
    private static final double HP_WAIT_FIRST_POSITION = 0;
    private static final double EXCESS_PATH_SPEED = 0.8;
    private static final double GATE_COLLECT_WAIT = 0.5;
    private static final double DETECTION_WAIT = 0.5;
    private static final double TURRET_DETECT_DEGREES = 7;
    private static final double TURRET_DETECT_DEGREES_2 = 13;

    // ========== POSES (far shot family from state_farblueoptimized) ==========
    private final Pose startPose           = new Pose(7 + 6.5, -7,    Math.toRadians(0));
    private final Pose farshotpose         = new Pose(12,      -17,   Math.toRadians(0));
    private final Pose ThirdPickupPose     = new Pose(60,      -35,   Math.toRadians(0));
    private final Pose midpoint2           = new Pose(8,      -38,   Math.toRadians(0));

    // Excess area poses from scenariofarblue
    private final Pose excessBallArea          = new Pose(66,  -35,  Math.toRadians(90));
    private final Pose excessBallAreaStrafeEnd = new Pose(66,  -9.8, Math.toRadians(90));
    private final Pose gateCollectDeepPose     = new Pose(68,  -44,  Math.toRadians(-63));  // actual collect position

    // HP collect poses (post-detection branch — duplicated from excess for independent tuning)
    private final Pose hpBallArea          = new Pose(66,  -18,  Math.toRadians(60));
    private final Pose hpBallAreaStrafeEnd = new Pose(67,  -9.8, Math.toRadians(10));

    // ========== PATHS ==========
    private PathChain ThirdLinePickupPath;
    private PathChain goBackPath;
    private PathChain excessPath;
    private PathChain excessPathStrafe;
    private PathChain gateDeepCollectPath;
    private PathChain hpPath;
    private PathChain hpPathStrafe;

    // ========== INIT ==========
    @Override
    public void init() {
        pathTimer             = new Timer();
        actionTimer           = new Timer();
        opmodeTimer           = new Timer();
        shootTimer            = new Timer();
        excessPathTimeoutTimer = new Timer();

        depo    = new Deposition_C(hardwareMap);
        LL      = new lifters(hardwareMap);
        intensitySensors = new ColorSensors_Intensity(hardwareMap);
        turret  = new TurretLimelight(hardwareMap);

        intake  = hardwareMap.get(DcMotor.class, "intake");
        d1      = hardwareMap.get(DcMotor.class, "depo");
        d2      = hardwareMap.get(DcMotor.class, "depo1");
        camTilt = hardwareMap.get(Servo.class,   "cam_tilt");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        LL.allDown();
        LL.set_angle_min();
        stopShooter();

        turret.resetTurretEncoder();
        turret.setDegreesTarget(98);

        if (camTilt != null) camTilt.setPosition(0.24);

        initAprilTag();

        ballCount           = 3;
        motifLocked         = false;
        motifDetectionCount = 0;
        detectedMotif       = "";

        telemetry.addLine("excess_farblue_turretdetect initialized | preload → third → excess → detect-in-place → branch");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();
        detectMotifFromAprilTags();

        telemetry.addData("Detected Motif", detectedMotif.isEmpty() ? "NONE YET" : detectedMotif);
        telemetry.addData("Detection Count", motifDetectionCount + "/" + MOTIF_DETECTION_THRESHOLD);
        telemetry.addData("Will Use", motif);
        telemetry.addData("Status", motifDetectionCount >= MOTIF_DETECTION_THRESHOLD ? "✓ Ready!" : "⚠ Waiting...");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        intensitySensors.calibrateAmbientFloor();
        motifLocked = true;

        if (detectedMotif.isEmpty()) {
            motif = "ppg";
            telemetry.addLine("⚠ No AprilTag detected — using default: ppg");
        } else {
            motif = detectedMotif;
            telemetry.addLine("✓ Locked motif: " + motif);
        }

        turret.setDegreesTarget(64);
        turret.setPid();

        shotCycleCount    = 0;
        excessPickupCount = 0;
        ballCount         = 3;

        telemetry.update();
        setPathState(0);
        setActionState(0);
    }

    @Override
    public void loop() {
        follower.update();
        turret.toTargetInDegrees();

        autonomousPathUpdate();
        autonomousActionUpdate();

        telemetry.addData("Path State",  pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Shot Cycle",   shotCycleCount);
        telemetry.addData("Ball Count",   ballCount);
        telemetry.addData("Motif",        motif + " (locked)");

        if (pathState == 50 || pathState == 51 || pathState == 511) {
            telemetry.addData("Scan Step",       pathState == 511 ? "ROI2 (right)" : "ROI1 (left)");
            telemetry.addData("ROI1 combined",   String.format("%.2f%%", ballCoverage.roi1CombinedPercent));
            telemetry.addData("ROI2 combined",   String.format("%.2f%%", ballCoverage.roi2CombinedPercent));
            telemetry.addData("Saved ROI1",      String.format("%.2f%%", savedFirstRoiCombined));
            if (pathState == 511) {
                telemetry.addData("DECISION", ballCoverage.roi2CombinedPercent > savedFirstRoiCombined ? "GATE COLLECT" : "HUMAN PLAYER");
            }
        }

        if (pathState == -1) {
            telemetry.addData("Auto Status", "Complete");
            telemetry.update();
            return;
        }

        telemetry.addData("X",       follower.getPose().getX());
        telemetry.addData("Y",       follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    // ========== APRILTAG VISION ==========
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(648.975, 648.975, 304.535, 221.714)
                .build();

        ballCoverage = new BallCoverageProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.addProcessor(ballCoverage);
        // Keep ball coverage processor off until the auto finishes pathing
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(ballCoverage, false);
    }

    private void detectMotifFromAprilTags() {
        if (motifLocked) return;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        String newMotif = "";

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.metadata.name.contains("Obelisk")) {
                if      (detection.id == 21) newMotif = "gpp";
                else if (detection.id == 22) newMotif = "ppg";
                else if (detection.id == 23) newMotif = "pgp";

                if (!newMotif.isEmpty()) {
                    if (newMotif.equals(detectedMotif)) {
                        motifDetectionCount++;
                    } else {
                        detectedMotif       = newMotif;
                        motifDetectionCount = 1;
                    }
                    if (motifDetectionCount >= MOTIF_DETECTION_THRESHOLD) {
                        motif = detectedMotif;
                    }
                    break;
                }
            }
        }
    }

    // ========== PATH STATE MACHINE ==========
    // Sequence: 0→1→101→2 (preload shot)
    //           → 20→21→211→22→23→24→25 (third line pickup + shot)
    //           → 30→31→311→32→321→33→34→35→36 (excess + shot)
    //           → 40→50→51 (turret detect in place → branch)
    //           → 52→521→522→53→54→55→56 (gate collect → return → shoot)
    //           → 60→61→611→62→621→63→64→65→66 (HP → return → shoot)

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===== PRELOAD =====
            case 0: // Spin up flywheel
                LL.set_angle_far_auto2();
                depo.setTargetVelocity(depo.ExcessBluePreload);
                SHOOT_INTERVAL = 0.43;
                setPathState(1);
                break;

            case 1: // Wait for flywheel
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    actionTimer.resetTimer();
                    setPathState(101);
                }
                break;

            case 101: // Settle before preload shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(2);
                }
                break;

            case 2: // Wait for preload shot to finish
                if (actionState == 0) {
                    ballCount = 0;
                    intake.setPower(-1);
                    SHOOT_INTERVAL = 0.43;
                    setPathState(20);
                }
                break;

            // ===== THIRD LINE PICKUP =====
            case 20: // Drive to third line
                intake.setPower(-1);
                buildThirdLinePickupPath();
                turret.setDegreesTarget(65);
                follower.followPath(ThirdLinePickupPath, true);
                setPathState(21);
                break;

            case 21: // Wait until third line reached
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    ballCount = 3;
                    actionTimer.resetTimer();
                    setPathState(211);
                }
                break;

            case 211: // Settle at third line before returning
                intake.setPower(-1);
                intensitySensors.update();
                if (intensitySensors.isFullRaw() || actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    if (intensitySensors.isFullRaw()) intake.setPower(1);
                    setPathState(22);
                }
                break;

            case 22: // Drive back to far shooting pose
                intake.setPower(1);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(23);
                break;

            case 23: // Arrived at shooting pose — spin up flywheel
                intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.ExcessBlue);
                    actionTimer.resetTimer();
                    setPathState(24);
                }
                break;

            case 24: // Settle before third line shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(25);
                }
                break;

            case 25: // Wait for third line shot to finish
                if (actionState == 0) {
                    ballCount = 0;
                    excessPickupCount = 0;
                    setPathState(30);
                }
                break;

            // ===== EXCESS (drive to area, strafe, then STOP) =====
            case 30: // Drive to hp area
                intake.setPower(-1);
                buildExcessPath();
                follower.setMaxPower(EXCESS_PATH_SPEED);
                follower.followPath(excessPath, true);
                excessPathTimeoutTimer.resetTimer();
                setPathState(31);
                break;

            case 31: // Wait until first position reached (or timeout)
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    actionTimer.resetTimer();
                    setPathState(311);
                }
                break;

            case 311: // Wait at first position — sensor-based early exit
                intake.setPower(-1);
                intensitySensors.update();
                if (intensitySensors.isFullRaw()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(312);
                } else if (actionTimer.getElapsedTimeSeconds() >= EXCESS_WAIT_FIRST_POSITION) {
                    buildExcessStrafePath();
                    follower.followPath(excessPathStrafe, true);
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 312: // spit briefly after detecting full at first excess position
                if (actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    buildExcessStrafePath();
                    follower.followPath(excessPathStrafe, true);
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 32: // Wait until strafe done
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    ballCount = 3;
                    actionTimer.resetTimer();
                    setPathState(321);
                }
                break;

            case 321: // Wait at second position — sensor-based early exit
                intake.setPower(-1);
                intensitySensors.update();
                if (intensitySensors.isFullRaw()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(322);
                } else if (actionTimer.getElapsedTimeSeconds() >= EXCESS_WAIT_SECOND_POSITION) {
                    follower.setMaxPower(1.0);
                    setPathState(33);
                }
                break;

            case 322: // spit briefly after detecting full at second excess position
                if (actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    follower.setMaxPower(1.0);
                    setPathState(33);
                }
                break;

            case 33: // Drive back to far shooting pose from excess
                intake.setPower(1);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(34);
                break;

            case 34: // Arrived at shooting pose from excess — spin up flywheel
                intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.ExcessBlue);
                    actionTimer.resetTimer();
                    setPathState(35);
                }
                break;

            case 35: // Settle before excess shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(36);
                }
                break;

            case 36: // Wait for excess shot to finish → detect in place
                if (actionState == 0) {
                    ballCount = 0;
                    excessPickupCount++;
                    setPathState(40);
                }
                break;

            // ===== TURRET DETECT IN PLACE (no driving — scan from current position) =====
            case 40: // Turn turret to detection angle and tilt camera
                intake.setPower(-1);
                turret.setDegreesTarget(TURRET_DETECT_DEGREES);
                if (camTilt != null) camTilt.setPosition(0.1667);
                setPathState(50);
                break;

            // ===== BLOB DETECTION & DECISION (two-step scan, mirrored from red) =====
            case 50: // Enable processor, capture ROI1 (left) at first turret angle
                intake.setPower(0);
                visionPortal.setProcessorEnabled(ballCoverage, true);
                actionTimer.resetTimer();
                setPathState(51);
                break;

            case 51: // Wait for ROI1 reading, save it, turn turret further right for ROI2
                if (actionTimer.getElapsedTimeSeconds() >= DETECTION_WAIT) {
                    savedFirstRoiCombined = ballCoverage.roi1CombinedPercent;
                    turret.setDegreesTarget(TURRET_DETECT_DEGREES_2);
                    actionTimer.resetTimer();
                    setPathState(511);
                }
                break;

            case 511: // Wait for ROI2 reading, then branch: ROI2 > saved ROI1 → gate, else → HP
                if (actionTimer.getElapsedTimeSeconds() >= DETECTION_WAIT) {
                    if (ballCoverage.roi2CombinedPercent > savedFirstRoiCombined) {
                        intake.setPower(-1);
                        actionTimer.resetTimer();
                        setPathState(52); // ROI2 more — collect from gate
                    } else {
                        setPathState(60); // ROI1 more — go to HP
                    }
                }
                break;

            // ===== GATE COLLECT → RETURN → SHOOT =====
            case 52: // Drive deeper into gate to actually collect
                intake.setPower(-1);
                buildGateDeepCollectPath();
                follower.followPath(gateDeepCollectPath, true);
                excessPathTimeoutTimer.resetTimer();
                setPathState(521);
                break;

            case 521: // Wait at deep collection position
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 2.0) {
                    ballCount = 3;
                    actionTimer.resetTimer();
                    setPathState(522);
                }
                break;

            case 522: // Hold at collection point — sensor-based early exit
                intake.setPower(-1);
                intensitySensors.update();
                if (intensitySensors.isFullRaw()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(523);
                } else if (actionTimer.getElapsedTimeSeconds() >= GATE_COLLECT_WAIT) {
                    setPathState(53);
                }
                break;

            case 523: // spit briefly after detecting full at gate collect
                if (actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    setPathState(53);
                }
                break;

            case 53: // Drive back to far shooting pose from gate
                intake.setPower(1);
                turret.setDegreesTarget(68);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(54);
                break;

            case 54: // Arrived at far shooting pose — spin up flywheel
                intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.ExcessBlue);
                    actionTimer.resetTimer();
                    setPathState(55);
                }
                break;

            case 55: // Settle before gate collect shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(56);
                }
                break;

            case 56: // Wait for gate collect shot to finish → scan again or done
                if (actionState == 0) {
                    ballCount = 0;
                    scanCycleCount++;
                    if (scanCycleCount < 2) {
                        setPathState(40); // Loop back for another scan cycle
                    } else {
                        depo.setTargetVelocity(0);
                        stopShooter();
                        setPathState(-1);
                    }
                }
                break;

            // ===== HP PATH → RETURN → SHOOT =====
            case 60: // Drive to HP area from current position
                intake.setPower(-1);
                buildHpPath();
                follower.setMaxPower(EXCESS_PATH_SPEED);
                follower.followPath(hpPath, true);
                excessPathTimeoutTimer.resetTimer();
                setPathState(61);
                break;

            case 61: // Wait until HP area reached (or timeout)
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    actionTimer.resetTimer();
                    setPathState(611);
                }
                break;

            case 611: // Wait at first HP position
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() >= HP_WAIT_FIRST_POSITION) {
                    buildHpStrafePath();
                    follower.followPath(hpPathStrafe, true);
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(62);
                }
                break;

            case 62: // Wait until HP strafe done
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    ballCount = 3;
                    actionTimer.resetTimer();
                    setPathState(621);
                }
                break;

            case 621: // Wait at second HP position — sensor-based early exit
                intake.setPower(-1);
                intensitySensors.update();
                if (intensitySensors.isFullRaw()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(622);
                } else if (actionTimer.getElapsedTimeSeconds() >= EXCESS_WAIT_SECOND_POSITION) {
                    follower.setMaxPower(1.0);
                    setPathState(63);
                }
                break;

            case 622: // spit briefly after detecting full at HP second position
                if (actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    follower.setMaxPower(1.0);
                    setPathState(63);
                }
                break;

            case 63: // Drive back to far shooting pose from HP
                intake.setPower(1);
                turret.setDegreesTarget(68);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(64);
                break;

            case 64: // Arrived at shooting pose from HP — spin up flywheel
                intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.ExcessBlue);
                    actionTimer.resetTimer();
                    setPathState(65);
                }
                break;

            case 65: // Settle before HP shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(66);
                }
                break;

            case 66: // Wait for HP shot to finish → scan again or done
                if (actionState == 0) {
                    ballCount = 0;
                    scanCycleCount++;
                    if (scanCycleCount < 2) {
                        setPathState(40); // Loop back for another scan cycle
                    } else {
                        depo.setTargetVelocity(0);
                        stopShooter();
                        setPathState(-1);
                    }
                }
                break;
        }
    }

    // ========== ACTION STATE MACHINE (SHOOTING) ==========
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0: // Idle
                break;

            case 1: // Initialize shooting
                LL.set_angle_far_auto2();
                depo.setTargetVelocity(depo.ExcessBlue);

                // Guard: only allow shooting from valid path states
                if (pathState != 2 && pathState != 25 && pathState != 36 && pathState != 101 && pathState != 24 && pathState != 35
                        && pathState != 55 && pathState != 56 && pathState != 65 && pathState != 66) {
                    break;
                }

                greenInSlot = getGreenPos();
                if (depo.reachedTargetHighTolerance()) {
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 2: // Wait for shooter to spin up
                depo.updatePID();

                if (pathState != 2 && pathState != 25 && pathState != 36 && pathState != 101 && pathState != 24 && pathState != 35
                        && pathState != 55 && pathState != 56 && pathState != 65 && pathState != 66) {
                    setActionState(0);
                    break;
                }

                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                }
                break;

            case 3: // Execute shooting sequence
                depo.updatePID();
                executeShootingSequence();

                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 3 + 0.15) {
                    LL.allDown();
                    depo.setTargetVelocity(0);
                    stopShooter();
                    shotCycleCount++;
                    setActionState(0);
                }
                break;
        }
    }

    // ========== SHOOTING HELPERS ==========
    private void executeShootingSequence() {
        if (motif.equals("gpp")) {
            if      (greenInSlot == 0) shootLRB();
            else if (greenInSlot == 1) shootRBL();
            else                       shootBLR();
        } else if (motif.equals("pgp")) {
            if      (greenInSlot == 0) shootBLR();
            else if (greenInSlot == 1) shootLRB();
            else                       shootRBL();
        } else { // ppg or default
            if      (greenInSlot == 0) shootRBL();
            else if (greenInSlot == 1) shootBLR();
            else                       shootLRB();
        }
    }

    private void shootLRB() {
        double t = shootTimer.getElapsedTimeSeconds();
        if      (t >= 0                      && t < SHOOT_INTERVAL - 0.05)  LL.leftUp();
        else if (t >= SHOOT_INTERVAL - 0.05  && t < SHOOT_INTERVAL)         LL.allDown();
        else if (t >= SHOOT_INTERVAL         && t < SHOOT_INTERVAL*2 - 0.05) LL.rightUp();
        else if (t >= SHOOT_INTERVAL*2 - 0.05 && t < SHOOT_INTERVAL*2)      LL.allDown();
        else if (t >= SHOOT_INTERVAL*2       && t < SHOOT_INTERVAL*3)       LL.backUp();
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if      (t >= 0                      && t < SHOOT_INTERVAL - 0.05)  LL.backUp();
        else if (t >= SHOOT_INTERVAL - 0.05  && t < SHOOT_INTERVAL)         LL.allDown();
        else if (t >= SHOOT_INTERVAL         && t < SHOOT_INTERVAL*2 - 0.05) LL.leftUp();
        else if (t >= SHOOT_INTERVAL*2 - 0.05 && t < SHOOT_INTERVAL*2)      LL.allDown();
        else if (t >= SHOOT_INTERVAL*2       && t < SHOOT_INTERVAL*3)       LL.rightUp();
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if      (t >= 0                      && t < SHOOT_INTERVAL - 0.05)  LL.rightUp();
        else if (t >= SHOOT_INTERVAL - 0.05  && t < SHOOT_INTERVAL)         LL.allDown();
        else if (t >= SHOOT_INTERVAL         && t < SHOOT_INTERVAL*2 - 0.05) LL.backUp();
        else if (t >= SHOOT_INTERVAL*2 - 0.05 && t < SHOOT_INTERVAL*2)      LL.allDown();
        else if (t >= SHOOT_INTERVAL*2       && t < SHOOT_INTERVAL*3)       LL.leftUp();
    }

    private int getGreenPos() {
        if (LL.sensors.getLeft()  == 1) return 0;
        if (LL.sensors.getRight() == 1) return 1;
        return 2;
    }

    // ========== PATH BUILDING ==========
    private void buildThirdLinePickupPath() {
        Pose cur = follower.getPose();
        ThirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint2, ThirdPickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), ThirdPickupPose.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    private void buildReturnToShootingPath() {
        Pose cur = follower.getPose();
        goBackPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, farshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), farshotpose.getHeading())
                .setTimeoutConstraint(0.1)
                .build();
    }

    private void buildExcessPath() {
        Pose cur = follower.getPose();
        excessPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, excessBallArea)))
                .setLinearHeadingInterpolation(cur.getHeading(), excessBallArea.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    private void buildExcessStrafePath() {
        Pose cur = follower.getPose();
        excessPathStrafe = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, excessBallAreaStrafeEnd)))
                .setLinearHeadingInterpolation(cur.getHeading(), excessBallAreaStrafeEnd.getHeading())
                .setTimeoutConstraint(1.2)
                .build();
    }

    private void buildGateDeepCollectPath() {
        Pose cur = follower.getPose();
        // add the midpoint later
//        Pose midpoint = new Pose(gateCollectDeepPose.getX(), cur.getY(), gateCollectDeepPose.getHeading());
        gateDeepCollectPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, gateCollectDeepPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), gateCollectDeepPose.getHeading())
                .setTimeoutConstraint(1.5)
//                .addPath(new Path(new BezierLine(midpoint, gateCollectDeepPose)))
//                .setConstantHeadingInterpolation(gateCollectDeepPose.getHeading())
//                .setTimeoutConstraint(1.5)
                .build();
    }

    private void buildHpPath() {
        Pose cur = follower.getPose();
        hpPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, hpBallArea)))
                .setLinearHeadingInterpolation(cur.getHeading(), hpBallArea.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    private void buildHpStrafePath() {
        Pose cur = follower.getPose();
        hpPathStrafe = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, hpBallAreaStrafeEnd)))
                .setLinearHeadingInterpolation(cur.getHeading(), hpBallAreaStrafeEnd.getHeading())
                .setTimeoutConstraint(1.2)
                .build();
    }

    // ========== UTILITY ===========
    private void stopShooter() {
        if (d1 != null) d1.setPower(0.0);
        if (d2 != null) d2.setPower(0.0);
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
        if (intake != null)       intake.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }

    // ========== BALL COVERAGE PROCESSOR ==========
    // Dual-ROI detection using YCrCb color thresholds with 5° rotation (mirrored from red).
    // ROI1 (left) and ROI2 (right) are evaluated independently.
    // Results are written on the vision thread and read on the main loop thread.
    static class BallCoverageProcessor implements VisionProcessor {

        // YCrCb ranges — (Y, Cr, Cb)
        private static final Scalar GREEN_LOWER  = new Scalar(32,  50,  118);
        private static final Scalar GREEN_UPPER  = new Scalar(255, 105, 145);
        private static final Scalar PURPLE_LOWER = new Scalar(32,  135, 135);
        private static final Scalar PURPLE_UPPER = new Scalar(255, 155, 169);

        // Rotation angle (positive = CCW, mirrored from red's -5°)
        private static final double ROTATE_DEGREES = -4.0;

        // ROI1 (left region / HP) fractional bounds
        private static final double ROI1_X_START = 0.0;
        private static final double ROI1_X_END   = 0.45;
        private static final double ROI1_Y_START = 0.35;
        private static final double ROI1_Y_END   = 0.65;

        // ROI2 (right region) fractional bounds
        private static final double ROI2_X_START = 0.45;
        private static final double ROI2_X_END   = 0.90;
        private static final double ROI2_Y_START = 0.35;
        private static final double ROI2_Y_END   = 0.70;

        volatile double roi1GreenPercent    = 0;
        volatile double roi1PurplePercent   = 0;
        volatile double roi1CombinedPercent = 0;
        volatile double roi2GreenPercent    = 0;
        volatile double roi2PurplePercent   = 0;
        volatile double roi2CombinedPercent = 0;

        private final Mat rgbMat      = new Mat();
        private final Mat rotatedMat  = new Mat();
        private final Mat ycrcbMat    = new Mat();
        private final Mat greenMask   = new Mat();
        private final Mat purpleMask  = new Mat();
        private final Mat combinedMat = new Mat();
        private final Mat roi1Mask    = new Mat();
        private final Mat roi2Mask    = new Mat();
        private final Mat tempMask    = new Mat();
        private final Mat vizMat      = new Mat();
        private final Mat grayRoi     = new Mat();
        private final Mat circlesMat  = new Mat();
        private Bitmap vizBitmap;
        private Mat kernel;
        private Mat rotMatrix;
        private int frameWidth;
        private int frameHeight;

        // Stored ROI corners for visualization
        private Point roi1TL, roi1BR, roi2TL, roi2BR;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            frameWidth  = width;
            frameHeight = height;
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

            Point center = new Point(width / 2.0, height / 2.0);
            rotMatrix = Imgproc.getRotationMatrix2D(center, ROTATE_DEGREES, 1.0);

            // ROI1 rectangle (left region)
            roi1TL = new Point(width * ROI1_X_START, height * ROI1_Y_START);
            roi1BR = new Point(width * ROI1_X_END,   height * ROI1_Y_END);
            roi1Mask.create(height, width, CvType.CV_8UC1);
            roi1Mask.setTo(new Scalar(0));
            Imgproc.rectangle(roi1Mask, roi1TL, roi1BR, new Scalar(255), -1);

            // ROI2 rectangle (right region)
            roi2TL = new Point(width * ROI2_X_START, height * ROI2_Y_START);
            roi2BR = new Point(width * ROI2_X_END,   height * ROI2_Y_END);
            roi2Mask.create(height, width, CvType.CV_8UC1);
            roi2Mask.setTo(new Scalar(0));
            Imgproc.rectangle(roi2Mask, roi2TL, roi2BR, new Scalar(255), -1);

            vizBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // 1. RGBA → RGB
            Imgproc.cvtColor(frame, rgbMat, Imgproc.COLOR_RGBA2RGB);

            // 2. Rotate
            Imgproc.warpAffine(rgbMat, rotatedMat, rotMatrix,
                    new Size(frameWidth, frameHeight),
                    Imgproc.INTER_LINEAR, Core.BORDER_REPLICATE, new Scalar(0));

            // 3. RGB → YCrCb
            Imgproc.cvtColor(rotatedMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

            // 4. Threshold
            Core.inRange(ycrcbMat, GREEN_LOWER,  GREEN_UPPER,  greenMask);
            Core.inRange(ycrcbMat, PURPLE_LOWER, PURPLE_UPPER, purpleMask);

            // 5. Morphological cleanup — MORPH_OPEN → MORPH_CLOSE → dilate×2 → erode×1
            Imgproc.morphologyEx(greenMask,  greenMask,  Imgproc.MORPH_OPEN,  kernel);
            Imgproc.morphologyEx(greenMask,  greenMask,  Imgproc.MORPH_CLOSE, kernel);
            Imgproc.dilate(greenMask,  greenMask,  kernel, new Point(-1, -1), 2);
            Imgproc.erode( greenMask,  greenMask,  kernel, new Point(-1, -1), 1);

            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN,  kernel);
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.dilate(purpleMask, purpleMask, kernel, new Point(-1, -1), 2);
            Imgproc.erode( purpleMask, purpleMask, kernel, new Point(-1, -1), 1);

            // 6. Combined mask
            Core.bitwise_or(greenMask, purpleMask, combinedMat);

            // 7. ROI1 coverage (percentage of whole image)
            double totalPixels = frameWidth * frameHeight;
            double roi1Total = Core.countNonZero(roi1Mask);
            if (roi1Total > 0) {
                Core.bitwise_and(greenMask,   roi1Mask, tempMask);
                roi1GreenPercent    = Core.countNonZero(tempMask) / totalPixels * 100.0;
                Core.bitwise_and(purpleMask,  roi1Mask, tempMask);
                roi1PurplePercent   = Core.countNonZero(tempMask) / totalPixels * 100.0;
                Core.bitwise_and(combinedMat, roi1Mask, tempMask);
                roi1CombinedPercent = Core.countNonZero(tempMask) / totalPixels * 100.0;
            }

            // 8. ROI2 coverage (percentage of whole image)
            double roi2Total = Core.countNonZero(roi2Mask);
            if (roi2Total > 0) {
                Core.bitwise_and(greenMask,   roi2Mask, tempMask);
                roi2GreenPercent    = Core.countNonZero(tempMask) / totalPixels * 100.0;
                Core.bitwise_and(purpleMask,  roi2Mask, tempMask);
                roi2PurplePercent   = Core.countNonZero(tempMask) / totalPixels * 100.0;
                Core.bitwise_and(combinedMat, roi2Mask, tempMask);
                roi2CombinedPercent = Core.countNonZero(tempMask) / totalPixels * 100.0;
            }

            // 9. Visualization: rotated frame + highlighted detections + ROI rectangles
            Imgproc.cvtColor(rotatedMat, vizMat, Imgproc.COLOR_RGB2RGBA);
            vizMat.setTo(new Scalar(0,   255,   0, 255), greenMask);
            vizMat.setTo(new Scalar(200,   0, 255, 255), purpleMask);

            Imgproc.rectangle(vizMat, roi1TL, roi1BR, new Scalar(255, 255, 0, 255), 2);
            Imgproc.rectangle(vizMat, roi2TL, roi2BR, new Scalar(0, 255, 255, 255), 2);

            // 10. Hough circle detection on ROI2 (right) combined mask
            Core.bitwise_and(combinedMat, roi2Mask, grayRoi);
            Mat houghKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));
            Imgproc.dilate(grayRoi, grayRoi, houghKernel);
            Imgproc.erode( grayRoi, grayRoi, houghKernel);
            Imgproc.HoughCircles(grayRoi, circlesMat, Imgproc.HOUGH_GRADIENT,
                    1.5, 30, 50, 20, 10, 80);
            for (int i = 0; i < circlesMat.cols(); i++) {
                double[] c = circlesMat.get(0, i);
                Point ctr = new Point(c[0], c[1]);
                int r = (int) Math.round(c[2]);
                Imgproc.circle(vizMat, ctr, r, new Scalar(0, 255, 255, 255), 2);
                Imgproc.circle(vizMat, ctr, 3, new Scalar(255, 0, 0, 255), -1);
                Imgproc.putText(vizMat,
                        String.format("(%d,%d) r=%d", (int) c[0], (int) c[1], r),
                        new Point(c[0] - 30, c[1] - r - 8),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4,
                        new Scalar(255, 255, 255, 255), 1);
            }

            Utils.matToBitmap(vizMat, vizBitmap);

            return vizBitmap;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasPxToImagePx,
                                Object userContext) {
            if (userContext instanceof Bitmap) {
                canvas.drawBitmap((Bitmap) userContext, null,
                        new RectF(0, 0, onscreenWidth, onscreenHeight), new Paint());
            }
        }
    }
}
