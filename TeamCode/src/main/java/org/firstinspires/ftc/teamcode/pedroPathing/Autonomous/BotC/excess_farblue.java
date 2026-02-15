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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * excess_farblue
 *
 * SCENARIO_4_DOMINATES sequence:
 *   Preload → Third line pickup → Shot → Excess → Shot → Gate collect #1 → Shot → Gate collect #2 → Shot → Get out
 *
 * Poses: farshotpose family from state_farblueoptimized
 * Excess area: from scenariofarblue
 */
@Autonomous(name = "excess_farblue", group = "Pedro")
public class excess_farblue extends OpMode {

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
    private Timer excessPathTimeoutTimer;

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int greenInSlot;
    private int excessPickupCount = 0;
    private int gateCollectCount = 0;
    private int shotCycleCount = 0;
    private int ballCount = 3;

    // ========== MOTIF ==========
    private String motif = "ppg";
    private String detectedMotif = "";
    private boolean motifLocked = false;
    private int motifDetectionCount = 0;
    private static final int MOTIF_DETECTION_THRESHOLD = 3;

    // ========== CONSTANTS ==========
    private static double SHOOT_INTERVAL = 0.375;
    private static final double SETTLE_TIME = 0.15;
    private static final double EXCESS_WAIT_FIRST_POSITION = 2.0;
    private static final double EXCESS_WAIT_SECOND_POSITION = 1.5;
    private static final double EXCESS_PATH_SPEED = 0.5;
    private static final double GATE_COLLECT_WAIT = 2.0;
    private static final double GATE_FIRST_WAIT = 1.5;
    private static final double GATE_STRAFE_SPEED = 0.85;
    private static final int TOTAL_GATE_COLLECTS = 2;

    // ========== POSES (far shot family from state_farblueoptimized) ==========
    private final Pose startPose           = new Pose(7 + 6.5, -7,    Math.toRadians(0));
    private final Pose farshotpose         = new Pose(12,      -17,   Math.toRadians(0));
    private final Pose ThirdPickupPose     = new Pose(56,      -35,   Math.toRadians(0));
    private final Pose midpoint2           = new Pose(8,      -38,   Math.toRadians(0));
    private final Pose outPose             = new Pose(30,      -17,   Math.toRadians(0));
    private final Pose collectFromGate     = new Pose(66,      -52,   Math.toRadians(-53));

    // Excess area poses from scenariofarblue

    //
    private final Pose excessBallArea          = new Pose(61,  -12,  Math.toRadians(10));
    private final Pose gateExcess1         = new Pose(68,  -12,  Math.toRadians(-40));
    private final Pose excessBallAreaStrafeEnd = new Pose(60,  -9.8, Math.toRadians(13));
    // ========== PATHS ==========
    private PathChain ThirdLinePickupPath;
    private PathChain goBackPath;
    private PathChain excessPath;
    private PathChain excessPathStrafe;
    private PathChain gateStrafePath;
    private PathChain getOut;
    private PathChain gateAreaPath2;

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
        sensors = new ColorSensors(hardwareMap);
        turret  = new TurretLimelight(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        d1     = hardwareMap.get(DcMotor.class, "depo");
        d2     = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        LL.allDown();
        LL.set_angle_min();
        stopShooter();

        turret.resetTurretEncoder();
        turret.setDegreesTarget(98);

        initAprilTag();

        ballCount           = 3;
        motifLocked         = false;
        motifDetectionCount = 0;
        detectedMotif       = "";

        telemetry.addLine("excess_farblue initialized | preload → third → excess → 2x gate collect");
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
        motifLocked = true;

        if (detectedMotif.isEmpty()) {
            motif = "ppg";
            telemetry.addLine("⚠ No AprilTag detected — using default: ppg");
        } else {
            motif = detectedMotif;
            telemetry.addLine("✓ Locked motif: " + motif);
        }

        turret.setDegreesTarget(65);
        turret.setPid();

        shotCycleCount    = 0;
        excessPickupCount = 0;
        gateCollectCount  = 0;
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
        telemetry.addData("Excess Done",  excessPickupCount);

        if (pathState == -1) {
            telemetry.addData("Auto Status", "Complete");
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

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
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
    // Flywheel only spins up at shooting position (not during driving/collecting)
    // Sequence: 0→1→101→2 (preload shot)
    //           → 20→21→211→22→23→24→25 (third line pickup + shot)
    //           → 30→31→311→32→321→33→34→35→36 (excess + shot)
    //           → 40→41→411→412→413→42→43→44→45 (gate collect #1 + shot)
    //           → 40→41→411→412→413→42→43→44→45 (gate collect #2 + shot)
    //           → 17→18→-1 (get out)

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===== PRELOAD =====
            case 0: // Spin up flywheel
                LL.set_angle_far_auto2();
                depo.setTargetVelocity(depo.farVeloblueauto);
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
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setPathState(22);
                }
                break;

            case 22: // Drive back to far shooting pose
                if (ballCount >= 3) intake.setPower(1);
                else                intake.setPower(0);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(23);
                break;

            case 23: // Arrived at shooting pose — spin up flywheel
                if (ballCount >= 3) intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.farVeloblueauto);
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

            // ===== EXCESS PICKUP =====
            case 30: // Drive to excess area at half speed
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

            case 311: // Wait at first position
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() >= EXCESS_WAIT_FIRST_POSITION) {
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

            case 321: // Wait at second position
                if (actionTimer.getElapsedTimeSeconds() >= EXCESS_WAIT_SECOND_POSITION) {
                    follower.setMaxPower(1.0);
                    setPathState(33);
                }
                break;

            case 33: // Drive back to far shooting pose from excess
                if (ballCount >= 3) intake.setPower(1);
                else                intake.setPower(0);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(34);
                break;

            case 34: // Arrived at shooting pose from excess — spin up flywheel
                if (ballCount >= 3) intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.farVeloblueauto);
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

            case 36: // Wait for excess shot to finish → go to gate collect
                if (actionState == 0) {
                    ballCount = 0;
                    excessPickupCount++;
                    gateCollectCount = 0;
                    setPathState(40);
                }
                break;

            // ===== GATE COLLECT (runs TOTAL_GATE_COLLECTS times) =====
            case 40: // Drive to gateExcess1 at half speed
                intake.setPower(-1);
                buildGateAreaPath();

                follower.followPath(gateAreaPath2, true);
                excessPathTimeoutTimer.resetTimer();
                setPathState(41);
                break;

            case 41: // Wait until gateExcess1 reached (or timeout)
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    actionTimer.resetTimer();
                    setPathState(411);
                }
                break;

            case 411: // Wait at gateExcess1 before strafing
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() >= GATE_FIRST_WAIT) {
                    buildGateStrafePath();

                    follower.followPath(gateStrafePath, true);
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(412);
                }
                break;

            case 412: // Wait until strafe to collectFromGate done
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    actionTimer.resetTimer();
                    setPathState(413);
                }
                break;

            case 413: // Wait at gate to collect balls
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() >= GATE_COLLECT_WAIT) {
                    ballCount = 3;
                    follower.setMaxPower(1.0);
                    setPathState(42);
                }
                break;

            case 42: // Drive back to far shooting pose from gate
                if (ballCount >= 3) intake.setPower(1);
                else                intake.setPower(0);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(43);
                break;

            case 43: // Arrived at shooting pose from gate — spin up flywheel
                if (ballCount >= 3) intake.setPower(1);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_far_auto2();
                    depo.setTargetVelocity(depo.farVeloblueauto);
                    actionTimer.resetTimer();
                    setPathState(44);
                }
                break;

            case 44: // Settle before gate collect shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(45);
                }
                break;

            case 45: // Wait for gate collect shot to finish
                if (actionState == 0) {
                    ballCount = 0;
                    gateCollectCount++;
                    if (gateCollectCount < TOTAL_GATE_COLLECTS) {
                        setPathState(40); // Loop back for another gate collect
                    } else {
                        buildGetOutPath();
                        setPathState(17);
                    }
                }
                break;

            // ===== GET OUT =====
            case 17:
                follower.followPath(getOut, true);
                setPathState(18);
                break;

            case 18:
                if (!follower.isBusy()) {
                    setPathState(-1);
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
                depo.setTargetVelocity(depo.farVeloblueauto);

                // Guard: only allow shooting from valid path states
                if (pathState != 2  && pathState != 25 && pathState != 36 &&
                        pathState != 45 && pathState != 101 && pathState != 24 &&
                        pathState != 35 && pathState != 44) {
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

                if (pathState != 2  && pathState != 25 && pathState != 36 &&
                        pathState != 45 && pathState != 101 && pathState != 24 &&
                        pathState != 35 && pathState != 44) {
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

    private void buildGateAreaPath() {
        Pose cur = follower.getPose();
        gateAreaPath2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, gateExcess1)))
                .setLinearHeadingInterpolation(cur.getHeading(), gateExcess1.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    private void buildGateStrafePath() {
        Pose cur = follower.getPose();
        gateStrafePath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, collectFromGate)))
                .setLinearHeadingInterpolation(cur.getHeading(), collectFromGate.getHeading())
                .setTimeoutConstraint(1.2)
                .build();
    }

    private void buildGetOutPath() {
        Pose cur = follower.getPose();
        getOut = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, outPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), outPose.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    // ========== UTILITY ==========
    private void stopShooter() {
        if (d1 != null) d1.setPower(0.0);
        if (d2 != null) d2.setPower (0.0);
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
}