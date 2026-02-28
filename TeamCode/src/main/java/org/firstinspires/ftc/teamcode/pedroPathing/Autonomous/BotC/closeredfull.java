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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Close Red Full", group = "Pedro")
public class closeredfull extends OpMode {

    // ========== SUBSYSTEMS ==========
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
    private  boolean hasThreeBalls = false;

    // ========== MODE TOGGLE ==========
    private boolean gateMode = false;
    private boolean prevTriangle = false;

    // ========== MODE-DEPENDENT SETTINGS ==========
    private double SHOOT_INTERVAL = 0.23;
    private int TOTAL_GATE_CYCLES = 2;

    // Normal mode values
    private static final double SHOOT_INTERVAL_NORMAL = 0.233;
    private static final int TOTAL_GATE_CYCLES_NORMAL = 2;

    // Gate mode values
    private static final double SHOOT_INTERVAL_GATE = 0.233;
    private static final int TOTAL_GATE_CYCLES_GATE = 3;

    // ========== CONSTANTS ==========
    private static final double SECOND_HOP_IN = 8;
    private static final double GATE_WAIT_TIME_FIRST = 0.93;
    private static final double GATE_WAIT_TIME_LATER = 0.675;
    private static final double SETTLE_TIME = 0.05;

    // ========== POSES ==========
    private final Pose startPose = new Pose(44, 128, Math.toRadians(35));
    private final Pose nearshotpose = new Pose(12, 81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, 81.5, Math.toRadians(34));

    private final Pose shotPoseInside = new Pose(13, 112, Math.toRadians(90));

    private final Pose firstPickupPose = new Pose(46, 81, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13.4, 58, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(10, 68, Math.toRadians(0));
    private final Pose secondpickuppose = new Pose(51.5, 55, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, 68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, 60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(54.3, 56.3, Math.toRadians(34));
    private final Pose back_lever = new Pose(54.3, 49.3, Math.toRadians(36.5));
    private final Pose outfromgate = new Pose(50, 55, Math.toRadians(42));
    private final Pose outfromgate1 = new Pose(50, 43, Math.toRadians(42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, 58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, 53, Math.toRadians(40));
    private final Pose intake_from_gate_rotate = new Pose(55, 54, Math.toRadians(0));
    private final Pose outPose = new Pose(21, 81.5, Math.toRadians(34));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain goBackPath1;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain firstLinePickupPath;
    private PathChain firstLineSecondHopPath;
    private PathChain thirdLinePickupPath;
    private PathChain gatebackPath;
    private PathChain getOut;
    private final Pose thirdLinePickupPose = new Pose(54, 33.5, Math.toRadians(0));
    private final Pose midpointToThird = new Pose(2, 30, Math.toRadians(0));

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
        turret.setRedAlliance();

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
        turret.setDegreesTarget(-105);

        initAprilTag();

        applyMode();

        telemetry.addLine("Close Red Full initialized - Triangle to toggle mode");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        detectMotifFromAprilTags();

        boolean currTriangle = gamepad1.triangle;
        if (currTriangle && !prevTriangle) {
            gateMode = !gateMode;
            applyMode();
        }
        prevTriangle = currTriangle;

        telemetry.addData("MODE", gateMode ? "GATE (3 gate cycles)" : "NORMAL (2 gates + third line)");
        telemetry.addData("Shoot Interval", SHOOT_INTERVAL);
        telemetry.addData("Gate Cycles", TOTAL_GATE_CYCLES);
        telemetry.addData("Motif Detected", motif);
        telemetry.addLine("Press TRIANGLE to toggle mode");
        telemetry.update();
    }

    private void applyMode() {
        if (gateMode) {
            SHOOT_INTERVAL = SHOOT_INTERVAL_GATE;
            TOTAL_GATE_CYCLES = TOTAL_GATE_CYCLES_GATE;
        } else {
            SHOOT_INTERVAL = SHOOT_INTERVAL_NORMAL;
            TOTAL_GATE_CYCLES = TOTAL_GATE_CYCLES_NORMAL;
        }
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(-47);
        turret.setPid();
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

        telemetry.addData("Mode", gateMode ? "GATE" : "NORMAL");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Shot Cycle", shotCycleCount);

        if (pathState >= 7 && pathState <= 11) {
            telemetry.addData("Gate Cycle", (gateHitCount + 1) + "/" + TOTAL_GATE_CYCLES);
        } else if (pathState >= 12 && pathState <= 16) {
            if (pathState == 13 || pathState == 14) {
                telemetry.addData("Sequence", "First Line - Second Hop");
            } else {
                telemetry.addData("Sequence", "First Line Pickup");
            }
        } else if ((pathState >= 20 && pathState <= 24) || pathState == 123) {
            telemetry.addData("Sequence", "Third Line Pickup");
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
                double yaw = detection.ftcPose.yaw;

                if (yaw > 40 && yaw < 90) {
                    if (detection.id == 21) motif = "pgp";
                    if (detection.id == 22) motif = "ppg";
                    if (detection.id == 23) motif = "gpp";
                } else if (yaw > -80 && yaw < -40) {
                    if (detection.id == 22) motif = "pgp";
                    if (detection.id == 23) motif = "ppg";
                    if (detection.id == 21) motif = "gpp";
                }
            }
        }
    }

    // ========== PATH STATE MACHINE ==========
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);
                buildGoBackPath();
                follower.followPath(goBackPath, true);
                setPathState(1);
                break;

            case 1:
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    setActionState(1);
                    setPathState(2);
                }
                break;

            case 2:
                if (actionState == 0) {
                    turret.setDegreesTarget(-13);
                    setPathState(3);
                }
                break;

            case 3:
                buildBezierPaths();
                intake.setPower(-1);
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);
                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5:
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(105);
                }
                break;

            case 105:
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    intake.setPower(0);
                    setPathState(6);
                }
                break;

            case 6:
                intake.setPower(0);
                if (actionState == 31) {
                    gateHitCount = 0;
                    setPathState(7);
                }
                break;

            // ===== GATE CYCLE LOOP =====
            case 7://inits path
                double waitTime = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                buildGatePaths(waitTime);
                intake.setPower(-1);
                follower.followPath(gateFirstPath, true);
                setPathState(8);
                break;

            case 8://does the first part of gate cycle to bump the gate
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.8){
                    actionTimer.resetTimer();
                    setPathState(99);
                }
                break;

            case 99://inits second path
                intake.setPower(-1);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;

            case 102://does the 2nd path of moving back
//                hasThreeBalls = checkThreeBalls();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.5) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9://waits at gate
                double waitTime2 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
//                hasThreeBalls = checkThreeBalls();
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);
                    buildGatePathsBack();
                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10:
                intake.setPower(1);
//                hasThreeBalls=false;
                depo.updatePID();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.5) {
                    actionTimer.resetTimer();
                    setPathState(110);
                }
                break;

            case 110:
                intake.setPower(0);
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(11);
                }
                break;

            case 11:
                if (actionState == 31) {
                    gateHitCount++;
                    if (gateHitCount < TOTAL_GATE_CYCLES) {
                        setPathState(7);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            // ===== FIRST LINE PICKUP =====
            case 12:
                LL.set_angle_close();
//                depo.setTargetVelocity(depo.closeVelo_New_auto);
                intake.setPower(-1);
                buildLinePickupPaths();
                follower.followPath(firstLinePickupPath, true);
                setPathState(13);
                break;

            case 13:
                depo.updatePID();
                if (!follower.isBusy()) {
//                    manageSecondHopIntake();
                    setPathState(14);
                }
                break;

            case 14:
                LL.set_angle_close();
                if (gateMode) {
                    depo.setTargetVelocity(depo.closeVelo_New_auto-80);
                    turret.setDegreesTarget(65);
                    buildReturnToShootingLastGate();
                    follower.followPath(goBackPath1, true);
                } else {
                    depo.setTargetVelocity(depo.closeVelo_New_auto+20);
                    buildReturnToShootingPath();
                    follower.followPath(goBackPath, true);
                }
                setPathState(15);
                break;

            case 15:
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(115);
                }
                break;

            case 115:
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    intake.setPower(0);
                    setPathState(16);
                }
                break;

            case 16:
                if (actionState == 31) {
                    intake.setPower(0);
                    if (gateMode) {
                        buildGetOutPath();
                        setPathState(17);
                    } else {
                        setPathState(20);
                    }
                }
                break;

            // ===== THIRD LINE PICKUP (normal mode only) =====
            case 20:
                intake.setPower(-1);
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto-120);
                buildThirdLinePickupPath();
                follower.followPath(thirdLinePickupPath, true);
                setPathState(21);
                break;

            case 21:
                depo.updatePID();
                if (!follower.isBusy()) {
//                    manageSecondHopIntake();
                    setPathState(22);
                }
                break;

            case 22:
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto-120);
                buildReturnToShootingLast();
                turret.setDegreesTarget(65);
                follower.followPath(goBackPath1, true);
                setPathState(23);
                break;

            case 23:
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(123);
                }
                break;

            case 123:
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    intake.setPower(0);
                    setPathState(24);
                }
                break;

            case 24:
                intake.setPower(0);
                if (actionState == 31) {
                    buildGetOutPath();
                    setPathState(17);
                }
                break;

            // ===== GET OUT =====
            case 17:
                intake.setPower(0);
//                follower.followPath(getOut, true);
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
            case 0://shot ends can start paths
                break;

            case 1://starting shooting
                LL.set_angle_close();
//                depo.setTargetVelocity(depo.closeVelo_New_auto);
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 2://waiting to reach flywheel speed
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                }
                break;

            case 3://shooting and turning shoot off
                depo.updatePID();
                boolean useRandomShooting = (shotCycleCount < 2);

                if (useRandomShooting) {
                    shootThreeRandom();
                } else {
                    executeShootingSequence();
                }

                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 2) {
                    setActionState(31);
                }
                break;
            case 31:
                depo.updatePID();
                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 3 + 0.25) {
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
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.backUp();
        }
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.rightUp();
        }
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.leftUp();
        }
    }

    private void shootThreeRandom() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.backUp();
        }
    }

    private int getGreenPos() {
        int pos = LL.sensors.getLeft();
        if (pos == 1) return 0;
        pos = LL.sensors.getRight();
        if (pos == 1) return 1;
        return 2;
    }

    // ========== PATH BUILDING METHODS ==========
    private void buildGoBackPath() {
        Pose cur = follower.getPose();
        goBackPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading(), 0.22)
                .setTimeoutConstraint(0.2)
                .build();
    }


    private void buildBezierPaths() {
        Pose cur = follower.getPose();
        bezierFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1, secondpickuppose)))
                .setLinearHeadingInterpolation(cur.getHeading(), secondpickuppose.getHeading(), 0.8)
                .build();

        bezierSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(secondpickuppose, midpoint2, nearshotpose2)))
                .setLinearHeadingInterpolation(secondpickuppose.getHeading(), nearshotpose2.getHeading(), 0.8)
                .addParametricCallback(0.65, () -> intake.setPower(1))
                .build();
    }

    private void buildGatePaths(double waitTime) {
        double yOffset = (gateHitCount == 2) ? 1.5 : gateHitCount * 0.5;
        Pose adjustedLever = new Pose(infront_of_lever_new.getX(), infront_of_lever_new.getY() + yOffset, infront_of_lever_new.getHeading());
        Pose adjustedBackLever = new Pose(back_lever.getX(), back_lever.getY() + yOffset, back_lever.getHeading());

        Pose cur = follower.getPose();
        gateFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, adjustedLever)))
                .setLinearHeadingInterpolation(cur.getHeading(), adjustedLever.getHeading(), 0.5)
                .setTimeoutConstraint(1)
                .build();

        gatebackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(adjustedLever, adjustedBackLever)))
                .setLinearHeadingInterpolation(adjustedBackLever.getHeading(), adjustedBackLever.getHeading(), 0.1)
                .setTimeoutConstraint(0.3)
                .build();
    }

    private void buildGatePathsBack() {
        Pose cur = follower.getPose();
        gateSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate1, nearshotpose2)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading(), 0.3)
                .build();
    }

    private void buildLinePickupPaths() {
        Pose cur = follower.getPose();
        firstLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, firstPickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), firstPickupPose.getHeading())
                .build();
    }

    private void buildThirdLinePickupPath() {
        Pose cur = follower.getPose();
        thirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpointToThird, thirdLinePickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), thirdLinePickupPose.getHeading(), 0.2)
                .setTimeoutConstraint(0.3)
                .build();
    }

    private void buildReturnToShootingPath() {
        Pose cur = follower.getPose();
        goBackPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose2)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading())
                .addParametricCallback(0.5, () -> intake.setPower(1))
                .build();
    }

    private void buildReturnToShootingLast() {
        Pose cur = follower.getPose();
        goBackPath1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, shotPoseInside)))
                .setLinearHeadingInterpolation(cur.getHeading(), shotPoseInside.getHeading())
                .addParametricCallback(0.4, () -> intake.setPower(1))
                .build();
    }

    private void buildReturnToShootingLastGate() {
        Pose cur = follower.getPose();
        goBackPath1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, shotPoseInside)))
                .setLinearHeadingInterpolation(cur.getHeading(), shotPoseInside.getHeading())
                .addParametricCallback(0.5, () -> intake.setPower(1))
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
    private boolean checkThreeBalls(){
//        if (intake == null || LL == null || sensors == null) return false;
        boolean allFull = (sensors.getRight() != 0 && sensors.getBack() != 0 && sensors.getLeft() != 0);
        return allFull;
    }

    // ========== UTILITY METHODS ==========
    private void manageSecondHopIntake() {
        if (intake == null || LL == null || sensors == null) return;

        boolean allFull = (sensors.getRight() != 0 && sensors.getBack() != 0 && sensors.getLeft() != 0);

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