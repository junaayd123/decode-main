package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

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

@Autonomous(name = "Closered optimized", group = "Pedro")
public class Closeredoptimized extends OpMode {

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
    private int shotCycleCount = 0;  // Tracks how many 3-ball cycles completed
    private boolean intakeRunning = false;  // ✅ ADD THIS

    // ========== CONSTANTS ==========
    private static final double SHOOT_INTERVAL = 0.335;
    private static final double SECOND_HOP_IN = 8;
    private static final double GATE_WAIT_TIME_FIRST = 1.6;
    private static final double GATE_WAIT_TIME_LATER = 1.2;
    private static final int TOTAL_GATE_CYCLES = 2;
    private static final double SETTLE_TIME = 0.3;  // ✅ ADD THIS - time to settle before shooting

    // ========== POSES ==========
    private final Pose startPose = new Pose(44, 128, Math.toRadians(35));
    private final Pose nearshotpose = new Pose(12, 81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, 81.5, Math.toRadians(34));
    private final Pose firstPickupPose = new Pose(46, 81, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13.4, 58, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(10, 68, Math.toRadians(0));
    private final Pose secondpickuppose = new Pose(56, 55, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, 68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, 60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(57.3, 56.3, Math.toRadians(34));
    private final Pose back_lever = new Pose(58.3, 52.3, Math.toRadians(36.5));
    private final Pose outfromgate = new Pose(50, 55, Math.toRadians(42));
    private final Pose outfromgate1 = new Pose(50, 43, Math.toRadians(42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, 58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, 53, Math.toRadians(40));
    private final Pose intake_from_gate_rotate = new Pose(55, 54, Math.toRadians(0));
    private final Pose outPose = new Pose(21, 81.5, Math.toRadians(34));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain firstLinePickupPath;
    private PathChain firstLineSecondHopPath;
    private PathChain thirdLinePickupPath;
    private PathChain gatebackPath;
    private PathChain getOut;
    private Pose thirdPickupPose;

    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();

        // Initialize subsystems
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


        // Initialize follower
        follower = C_Bot_Constants.createFollower(hardwareMap);


        follower.setStartingPose(startPose);

        // Initialize launcher
        LL.allDown();
        LL.set_angle_min();
        stopShooter();

        // Initialize turret
        turret.resetTurretEncoder();
        turret.setDegreesTarget(-105);

        // Initialize AprilTag vision
        initAprilTag();

        telemetry.addLine("State-based Auto initialized (Webcam) - OPTIMIZED");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        // Detect motif from AprilTags using webcam
        detectMotifFromAprilTags();

        telemetry.addData("Motif Detected", motif);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(-44.5);
        turret.setPid();
        shotCycleCount = 0;
        setPathState(0);
        setActionState(0);
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
        if (pathState == -1){
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
                // Check yaw angle to determine which face we're looking at
                double yaw = detection.ftcPose.yaw;

                // Using red side logic (blueSide = false)
                if (yaw > 40 && yaw < 90) {
                    // First check position
                    if (detection.id == 21) motif = "pgp";
                    if (detection.id == 22) motif = "ppg";
                    if (detection.id == 23) motif = "gpp";
                } else if (yaw > -80 && yaw < -40) {
                    // Second check position
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
            case 0: // Go back to near shot pose - START FLYWHEEL
                // ✅ Start spinning flywheel at the very beginning
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                buildGoBackPath();
                follower.followPath(goBackPath, true);
                setPathState(1);
                break;

            case 1: // Wait to reach near shot pose
                depo.updatePID();  // ✅ Keep updating PID
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(101);  // ✅ Go to new settling state
                }
                break;

            case 101: // ✅ NEW STATE - Settle before first shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1); // Start shooting
                    setPathState(2);
                }
                break;

            case 2: // Wait for shooting to complete
                if (actionState == 0) { // Shooting done
                    turret.setDegreesTarget(-15);
                    setPathState(3);
                }
                break;

            case 3: // Bezier curve pickup - first path
                buildBezierPaths();
                intake.setPower(-1);
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4: // Wait for first bezier path
                if (!follower.isBusy()) {

                    // ✅ Start spinning flywheel BEFORE next path
                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);

                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for second bezier path
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(105);  // ✅ Go to settling state
                }
                break;
            case 105: // ✅ NEW STATE - Settle before second shot
                intake.setPower(1);
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooting cycle 2
                intake.setPower(0);
                if (actionState == 0) {
                    gateHitCount = 0; // Reset counter
                    setPathState(7); // Start gate cycles
                }
                break;
            //

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
                double waitTime1 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;

                intake.setPower(-1);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;
            case 102: // Gate - wait at back_lever position
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect artifacts
                double waitTime2 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {

                    // ✅ Start spinning flywheel BEFORE return path
                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);
                    buildGatePathsBack();

                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10: // Gate - return to shooting position
                intake.setPower(1);
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(110);  // ✅ Go to settling state
                }
                break;

            case 110: // ✅ NEW STATE - Settle before gate shot
                intake.setPower(0);
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(11);
                }
                break;

            case 11: // Wait for shooting to complete
                if (actionState == 0) {
                    gateHitCount++;

                    if (gateHitCount < TOTAL_GATE_CYCLES) {
                        setPathState(7); // Loop back to gate cycle
                    } else {
                        setPathState(12); // Move to first line pickup
                    }
                }
                break;

            // ===== FIRST LINE PICKUP =====
            case 12: // Drive straight to first line pickup
                // ✅ Start spinning flywheel BEFORE going to pickup
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                intake.setPower(-1);
                buildLinePickupPaths();
                follower.followPath(firstLinePickupPath, true);
                setPathState(13);
                break;

            case 13: // Wait until pickup reached
                depo.updatePID();  // ✅ Keep updating PID during drive
                if (!follower.isBusy()) {
                    manageSecondHopIntake();
                    setPathState(14);
                }
                break;

            case 14: // Drive straight back to shooting pose
                // ✅ Start spinning flywheel BEFORE return path
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(15);
                break;

            case 15: // Wait until back at shooting pose
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(115);  // ✅ Go to settling state
                }
                break;

            case 115: // ✅ NEW STATE - Settle before final shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);

                    setPathState(16);
                }
                break;

            case 16: // Final shooting sequence
                if (actionState == 0) {

                    intake.setPower(1);
                    buildGetOutPath();
                    setPathState(17);
                }
                break;
            case 17:
                intake.setPower(0);
                follower.followPath(getOut, true);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    setPathState(-1); // Auto complete
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
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                // ✅ Check if already at speed (from pre-spinning)
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);  // Skip wait, go straight to shooting!
                } else {
                    setActionState(2);  // Still need to wait
                }
                break;

            case 2: // Wait for shooter to spin up
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                }
                break;

            case 3: // Execute shooting sequence
                depo.updatePID();

                // Use random shooting for first 2 cycles (6 balls), then motif
                boolean useRandomShooting = (shotCycleCount < 2);

                if (useRandomShooting) {
                    shootThreeRandom();
                } else {
                    executeShootingSequence();
                }

                if (shootTimer.getElapsedTimeSeconds() > SHOOT_INTERVAL * 3) {
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
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.backUp();
        }
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.rightUp();
        }
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.leftUp();
        }
    }

    private void shootThreeRandom() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) {
            LL.allDown();  // 50ms to retract before next ball
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) {
            LL.allDown();  // 50ms to retract before next ball
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
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
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
                .build();
    }

    private void buildGatePaths(double waitTime) {
        Pose cur = follower.getPose();
        gateFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, infront_of_lever_new)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever_new.getHeading(), 0.5)
                .setTimeoutConstraint(1.1)
                .build();

        gatebackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(infront_of_lever_new, back_lever)))
                .setLinearHeadingInterpolation(back_lever.getHeading(), back_lever.getHeading(), 0.1)
                .setTimeoutConstraint(0.1)
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

    private void buildReturnToShootingPath() {
        Pose cur = follower.getPose();
        goBackPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose2)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading())
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

    // ========== UTILITY METHODS ==========
    private void manageSecondHopIntake() {
        if (intake == null || LL == null || sensors == null) return;

        // Check if all slots are full
        boolean allFull = (sensors.getRight() != 0 && sensors.getBack() != 0 && sensors.getLeft() != 0);

        if (intakeRunning) {
            if (allFull) {
                // All slots full - trigger reverse sequence
                actionTimer.resetTimer();
                intakeRunning = false;
            }
        } else {
            // Not currently intaking - check if we should start
            if (!allFull) {
                intake.setPower(-1);
                intakeRunning = true;
            }
        }

        // Handle reverse sequence when full (like reverseIntake() in teleop)
        if (!intakeRunning && actionTimer.getElapsedTimeSeconds() < 0.5 && actionTimer.getElapsedTimeSeconds() > 0) {
            intake.setPower(1); // Reverse for 0.5 seconds
        } else if (!intakeRunning && actionTimer.getElapsedTimeSeconds() >= 0.5) {
            intake.setPower(0); // Stop after reverse
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