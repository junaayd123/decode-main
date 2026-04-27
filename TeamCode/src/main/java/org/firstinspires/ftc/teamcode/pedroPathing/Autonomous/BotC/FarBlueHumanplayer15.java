
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.regressions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "FarBluehuman 15", group = "Pedro")
public class FarBlueHumanplayer15 extends OpMode {

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
    private static double SHOOT_INTERVAL = 0.23;
    private static final double SECOND_HOP_IN = 6.5;
    private static final double GATE_WAIT_TIME_FIRST = 1.0;
    private static final double GATE_WAIT_TIME_LATER = 0.8;
    private static final int TOTAL_GATE_CYCLES = 1;
    private static final double SETTLE_TIME = 0.17;  // ✅ NEW - time to settle before shooting
    private static final double SPIT_DURATION_SEC = 0.5; // max spit timeout — exits early if sensors clear

    // ========== POSES ==========
    private final Pose startPose = new Pose(7+6.5, -7, Math.toRadians(0));
    private final Pose nearshotpose = new Pose(12, -81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, -81.5, Math.toRadians(34));
    private final Pose ThirdPickupPose = new Pose(59, -35, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13, -60, Math.toRadians(0));
    private final Pose farshotpose = new Pose(12, -17, Math.toRadians(0));

    private final Pose outPose = new Pose(30, -17, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(23, -35, Math.toRadians(0));
    private final Pose midpoint3 = new Pose(21, -61, Math.toRadians(0));
    private final Pose secondLinePickupPose = new Pose(65, -62, Math.toRadians(0));
    private final Pose secondpickupPose = new Pose(56, -38, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, -68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, -60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(62, -62, Math.toRadians(-34));
    private final Pose back_lever = new Pose(63, -54, Math.toRadians(-38));
    private final Pose infront_of_lever_adj = new Pose(60.5, -61, Math.toRadians(-34));
    private final Pose excessBallArea = new Pose (72, -30, Math.toRadians(90));
    private final Pose excessBallAreaStrafeEnd = new Pose(70, -7.3, Math.toRadians(90));
    // private final Pose outfromgate = new Pose(50, 50, Math.toRadians(42));
    //  private final Pose midpointbefore_intake_from_gate = new Pose(52, 58, Math.toRadians(0));
//    private final Pose intake_from_gate = new Pose(56, 53, Math.toRadians(40));
    //   private final Pose intake_from_gate_rotate = new Pose(55, 54, Math.toRadians(0));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain getOut;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain ThirdLinePickupPath;
    private PathChain firstLineSecondHopPath;
    private PathChain gatebackPath;
    private PathChain excessPath;
    private PathChain excessPathStrafe;
    private Timer excessPathTimeoutTimer;
    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        excessPathTimeoutTimer = new Timer();

        // Initialize subsystems
        depo = new Deposition_C(hardwareMap);
        LL = new lifters(hardwareMap);
        sensors = new ColorSensors(hardwareMap);
        turret = new TurretLimelight(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        d1 = hardwareMap.get(DcMotor.class, "depo");
        d2 = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize follower
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize launcher
        LL.allDown();
        LL.set_angle_min();
        stopShooter();

        // Initialize turret
        turret.resetTurretEncoder();

        turret.setDegreesTarget(98);
        //

        // Initialize AprilTag vision
        initAprilTag();

        telemetry.addLine("State-based Auto initialized (Webcam) - OPTIMIZED");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        // Detect motif from  using webcam
        detectMotifFromAprilTags();

        telemetry.addData("Motif Detected", motif);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(73.4);
        turret.setPid();
        shotCycleCount = 0;
        setPathState(0);
        setActionState(0);
        regressions.motif = motif;
    }


    @Override
    public void loop() {
        // Update follower and subsystems
        follower.update();
        turret.toTargetInDegrees();

        // Run state machines
        autonomousPathUpdate();
        autonomousActionUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Shot Cycle", shotCycleCount);

        // Show current cycle based on state
        if (pathState >= 7 && pathState <= 11) {
            telemetry.addData("Gate Cycle", (gateHitCount + 1) + "/" + TOTAL_GATE_CYCLES);
        } else if (pathState >= 12 && pathState <= 17) {
            if (pathState == 13 || pathState == 14) {
                telemetry.addData("Sequence", "First Line - Second Hop");
            } else {
                telemetry.addData("Sequence", "First Line Pickup");
            }
        }

        if (pathState == -1) {
            telemetry.addData("Auto Status", "Complete");
            return;
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Motif", motif);
        telemetry.update();
    }

    // ========== APRILTAG VISION METHODS ==========
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
            case 0: // Spin up flywheel for preload
                LL.set_angle_farredoptimized();
                depo.setTargetVelocity(depo.ExcessRedPreload - 80);
                SHOOT_INTERVAL = 0.335;
                setPathState(1);
                break;

            case 1: // Wait for flywheel to spin up — timeout at 1.2s
                depo.updatePID();
                intake.setPower(-1); // start intake early during spinup
                if (depo.reachedTargetHighTolerance() || pathTimer.getElapsedTimeSeconds() > 1.2) {
                    actionTimer.resetTimer();
                    setPathState(101);
                }
                break;

            case 101: // Settle before first shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(2);
                }
                break;

            case 2: // Start driving immediately when shot is done
                if (actionState == 0) {
                    SHOOT_INTERVAL = 0.335;
                    buildBezierPaths();
                    intake.setPower(-1);
                    follower.followPath(bezierFirstPath, true);
                    setPathState(4); // skip case 3, jump straight to waiting on the path
                }
                break;

            case 3: // (unused — kept to avoid gaps)
                setPathState(4);
                break;

            case 4: // Wait for first bezier path
                if (!follower.isBusy()) {
                    LL.set_angle_farredoptimized();
                    depo.setTargetVelocity(depo.ExcessRed - 80);
                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for second bezier path
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(105);
                }
                break;

            case 105: // Settle before second shot
                intake.setPower(1);
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(101);
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooting cycle 2
                intake.setPower(0);
                if (actionState == 0) {
                    gateHitCount = 0;
                    setPathState(7);
                }
                break;

            // ===== GATE CYCLE LOOP =====
            case 7: // Gate - go to gate
                double waitTime = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                buildGatePaths(waitTime);
                intake.setPower(-1);
                follower.followPath(gateFirstPath, true);
                setPathState(8);
                break;

            case 8: // Gate - wait at gate position
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(99);
                }
                break;

            case 99: // Gate - go to back_lever
                intake.setPower(-1);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;

            case 102: // Gate - wait at back_lever position
                if (!follower.isBusy()) {
                    buildGatePathBack(GATE_WAIT_TIME_FIRST);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect
                double waitTime2 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    LL.set_angle_farredoptimized();
                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10: // Gate - return to shooting position
                intake.setPower(0);
                intake.setPower(1);
                depo.setTargetVelocity(depo.ExcessRed - 80);
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(110);
                }
                break;

            case 110: // Settle before gate shot
                intake.setPower(0);
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(101);
                    setPathState(11);
                }
                break;

            case 11: // Wait for shooting to complete
                if (actionState == 0) {
                    gateHitCount++;
                    if (gateHitCount < TOTAL_GATE_CYCLES) {
                        setPathState(7);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            // ===== THIRD LINE PICKUP =====
            case 12:
                LL.set_angle_farredoptimized();
                depo.setTargetVelocity(depo.ExcessRed - 80);
                intake.setPower(-1);
                follower.followPath(ThirdLinePickupPath, true);
                setPathState(13);
                break;

            case 13:
                depo.updatePID();
                if (!follower.isBusy()) {
                    buildReturnToShootingPath();
                    LL.set_angle_farredoptimized();
                    depo.setTargetVelocity(depo.ExcessRed - 80);
                    follower.followPath(goBackPath, true);
                    setPathState(14);
                }
                break;

            case 14:
                depo.updatePID();
                intake.setPower(1);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(115);
                }
                break;

            case 115: // Settle before third-line shot
                depo.updatePID();
                intake.setPower(1);
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    intake.setPower(0);
                    setActionState(101);
                    setPathState(15);
                }
                break;

            case 15: // Start driving to excess area as last ball is fired
                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 2) {
                    buildExcessPath();
                    follower.followPath(excessPath, true);
                    intake.setPower(-1);
                    setPathState(149);
                }
                break;

            case 149: // Wait until arrived at excess area (merged 149+150)
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(151);
                }
                break;

            case 151: // Wait at first excess position — sensor-based early exit
                intake.setPower(-1);
                if (isRobotFull()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(1511);
                } else if (actionTimer.getElapsedTimeSeconds() > 0.2) { // reduced from 0.6s
                    buildExcessStrafePath();
                    follower.followPath(excessPathStrafe, true);
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(152);
                }
                break;

            case 1511: // Full at first position — spit then skip strafe, go straight back
                if (!isRobotFull() || actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    buildReturnToShootingPath(); // skip strafe entirely
                    follower.followPath(goBackPath, true);
                    setPathState(154);
                }
                break;

            case 152: // Wait until strafe done
                intake.setPower(-1);
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    actionTimer.resetTimer();
                    setPathState(153);
                }
                break;

            case 153: // Wait at second excess position — sensor-based early exit
                intake.setPower(-1);
                if (isRobotFull()) {
                    intake.setPower(1); // ✅ spit
                    actionTimer.resetTimer();
                    setPathState(1531); // ✅ go to spit state
                } else if (actionTimer.getElapsedTimeSeconds() > 0.2) {
                    buildReturnToShootingPath();
                    intake.setPower(1);
                    follower.followPath(goBackPath, true);
                    setPathState(154);
                }
                break;

            case 1531: // Spit until sensors clear or timeout
                if (!isRobotFull() || actionTimer.getElapsedTimeSeconds() > SPIT_DURATION_SEC) {
                    intake.setPower(0);
                    buildReturnToShootingPath();
                    follower.followPath(goBackPath, true);
                    setPathState(154);
                }
                break;

            case 154:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_farredoptimized();
                    depo.setTargetVelocity(depo.ExcessRed - 80);
                    actionTimer.resetTimer();
                    setPathState(155);
                }
                break;

            case 155:
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(101);
                    setPathState(156);
                }
                break;

            case 156:
                if (actionState == 0) {
                    LL.allDown();
                    depo.setTargetVelocity(0);
                    stopShooter();
                    buildGetOutPath();
                    setPathState(17);
                }
                break;

            case 19: // small settle
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() > 0.2) {
                    buildReturnToShootingPath();
                    intake.setPower(1);
                    follower.followPath(goBackPath, true);
                    setPathState(20);
                }
                break;

            case 20: // return to shooting
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    LL.set_angle_farredoptimized();
                    depo.setTargetVelocity(depo.ExcessRed - 80);
                    actionTimer.resetTimer();
                    setPathState(21);
                }
                break;

            case 21: // settle before final shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(101);
                    setPathState(22);
                }
                break;

            case 22: // final shot
                if (actionState == 0) {
                    LL.allDown();
                    depo.setTargetVelocity(0);
                    stopShooter();
                    buildGetOutPath();
                    setPathState(17);
                }
                break;

            case 16: // Final shooting sequence
                intake.setPower(0);
                if (actionState == 0) {
                    setPathState(17);
                    buildGetOutPath();
                }
                break;

            case 17:
                buildGetOutPath();
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
            case 0:
                break;

            case 1:
                LL.set_angle_farredoptimized();
                depo.setTargetVelocity(depo.ExcessRedPreload);
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 101:
                LL.set_angle_farredoptimized();
                depo.setTargetVelocity(depo.ExcessRed);
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 2:
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                }
                break;

            case 3:
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

    // ========== SHOOTING HELPER METHODS ==========
    private void executeShootingSequence() {
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

    private void shootLRB() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05)              LL.leftUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.rightUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.backUp();
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05)              LL.backUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.leftUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.rightUp();
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05)              LL.rightUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.backUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.leftUp();
    }

    private void shootThreeRandom() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05)              LL.leftUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.rightUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.backUp();
    }

    private int getGreenPos() {
        int pos = LL.sensors.getLeft();
        if (pos == 1) return 0;
        pos = LL.sensors.getRight();
        if (pos == 1) return 1;
        return 2;
    }

    // ✅ NEW — helper to check if all sensor slots are full
    private boolean isRobotFull() {
        return sensors.getRight() != 0 && sensors.getBack() != 0 && sensors.getLeft() != 0;
    }

    // ========== PATH BUILDING METHODS ==========
    private void buildGetOutPath() {
        Pose cur = follower.getPose();
        getOut = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, outPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), outPose.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    private void buildBezierPaths() {
        Pose cur = follower.getPose();
        bezierFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1, secondLinePickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), secondLinePickupPose.getHeading(), 0.5)
                .build();

        bezierSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(secondLinePickupPose, midpoint1, farshotpose)))
                .setLinearHeadingInterpolation(secondLinePickupPose.getHeading(), farshotpose.getHeading(), 0.5)
                .setTimeoutConstraint(0.1)
                .build();

        ThirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(midpoint2, ThirdPickupPose)))
                .setLinearHeadingInterpolation(midpoint2.getHeading(), ThirdPickupPose.getHeading())
                .build();
    }

    private void buildGatePaths(double waitTime2) {
        Pose cur = follower.getPose();
        gateFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint3, infront_of_lever_new, infront_of_lever_adj)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever_new.getHeading(), 0.5)
                .setTimeoutConstraint(0.2)
                .build();

        gatebackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(infront_of_lever_new, back_lever)))
                .setLinearHeadingInterpolation(back_lever.getHeading(), back_lever.getHeading(), 0.1)
                .setTimeoutConstraint(0.3)
                .build();
    }

    private void buildGatePathBack(double waitTime) {
        Pose cur = follower.getPose();
        gateSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint3, farshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), farshotpose.getHeading(), 0.3)
                .setTimeoutConstraint(0.1)
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

    // ========== UTILITY METHODS ==========
    private void manageSecondHopIntake() {
        if (intake == null || LL == null || sensors == null) return;
        boolean allFull = isRobotFull();

        if (intakeRunning) {
            if (allFull) {
                actionTimer.resetTimer();
                intakeRunning = false;
            }
        } else {
            if (!allFull) {
                intake.setPower(-1);
                intakeRunning = true;
            }
        }

        if (!intakeRunning && actionTimer.getElapsedTimeSeconds() < 0.5 && actionTimer.getElapsedTimeSeconds() > 0) {
            intake.setPower(1);
        } else if (!intakeRunning && actionTimer.getElapsedTimeSeconds() >= 0.5) {
            intake.setPower(0);
        }
    }

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
        if (intake != null) intake.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }
}