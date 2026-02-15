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

@Autonomous(name = "state_Farblue optimized", group = "Pedro")
public class state_farblueoptimized extends OpMode {

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
    private String motif = "ppg"; // ✅ DEFAULT to "ppg" if no detection
    private String detectedMotif = ""; // ✅ Track what was actually detected
    private int gateHitCount = 0;
    private int shotCycleCount = 0;
    private boolean intakeRunning = false;
    private int ballCount = 3;

    // ======== CONSTANTS ==========
    private static double SHOOT_INTERVAL = 0.4;
    private static final double SECOND_HOP_IN = 8;
    private static final double GATE_WAIT_TIME_FIRST = 0.8;
    private static final double GATE_WAIT_TIME_LATER = 0.6;
    private static final int TOTAL_GATE_CYCLES = 2;
    private static final double SETTLE_TIME = 0.10;

    // ========== POSES ==========
    private final Pose startPose = new Pose(7+6.5, -7, Math.toRadians(0));
    private final Pose nearshotpose = new Pose(12, -81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, -81.5, Math.toRadians(-34));
    private final Pose ThirdPickupPose = new Pose(56, -35, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(15, -66, Math.toRadians(0));
    private final Pose farshotpose = new Pose(12, -17, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(22, -36, Math.toRadians(0));
    private final Pose midpoint3 = new Pose(19, -47, Math.toRadians(0));
    private final Pose secondLinePickupPose = new Pose(59.5, -62, Math.toRadians(0));
    private final Pose secondpickupPose = new Pose(56, -38, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, -68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, -60, Math.toRadians(0));
    private final Pose lever_touch = new Pose(60, -60.5, Math.toRadians(-32));
    private final Pose back_lever = new Pose(59, -54, Math.toRadians(-38.5));
    private final Pose outPose = new Pose(30, -17, Math.toRadians(0));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain ThirdLinePickupPath;
    private PathChain firstLineSecondHopPath;
    private PathChain gatebackPath;
    private PathChain getOut;

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
        turret.setDegreesTarget(100);

        // Initialize AprilTag vision
        initAprilTag();

        ballCount = 3;

        telemetry.addLine("State-based Auto initialized (Webcam) - MOTIF FIXED");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        // ✅ Continuously detect motif during init_loop
        detectMotifFromAprilTags();

        telemetry.addData("Detected Motif", detectedMotif.isEmpty() ? "NONE YET" : detectedMotif);
        telemetry.addData("Will Use", motif);
        telemetry.addData("Status", detectedMotif.isEmpty() ? "⚠ Waiting for AprilTag..." : "✓ Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(66.8);
        turret.setPid();
        shotCycleCount = 0;
        ballCount = 3;

        // ✅ Lock in the final motif (or keep default if nothing detected)
        if (detectedMotif.isEmpty()) {
            telemetry.addLine("⚠ WARNING: No AprilTag detected! Using default motif: " + motif);
        } else {
            telemetry.addLine("✓ Locked motif: " + motif);
        }
        telemetry.update();

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
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Active Motif", motif); // ✅ Show active motif

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
                // ✅ Update both detected and active motif
                if (detection.id == 21) {
                    motif = "gpp";
                    detectedMotif = "gpp";
                } else if (detection.id == 22) {
                    motif = "pgp";
                    detectedMotif = "pgp";
                } else if (detection.id == 23) {
                    motif = "ppg";
                    detectedMotif = "ppg";
                }
            }
        }
    }

    // ========== PATH STATE MACHINE ==========
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start - spin up flywheel
                LL.set_angle_far_auto();
                depo.setTargetVelocity(depo.farVeloblueauto);
                SHOOT_INTERVAL = 0.4;
                setPathState(1);
                break;

            case 1: // Wait for flywheel to spin up
                depo.updatePID();
                if (depo.reachedTargetHighTolerance()) {
                    actionTimer.resetTimer();
                    setPathState(101);
                }
                break;

            case 101: // Settle before first shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1); // Start shooting
                    setPathState(2);
                }
                break;

            case 2: // Wait for shooting to complete
                if (actionState == 0) { // Shooting done
                    ballCount = 0;
                    intake.setPower(-1); // Start intake
                    SHOOT_INTERVAL = 0.4;
                    setPathState(3);
                }
                break;

            case 3: // Bezier curve pickup - first path
                buildBezierPaths();
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4: // Wait for first bezier path (picking up balls)
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    ballCount = 3;
                    LL.set_angle_far_auto();
                    depo.setTargetVelocity(depo.farVeloblueauto);


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
                depo.updatePID();

                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    intake.setPower(1);
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooting cycle 2
                if (actionState == 0) {
                    ballCount = 0;
                    gateHitCount = 0;
                    intake.setPower(0);
                    setPathState(7);

                }

                break;

            // ===== GATE CYCLE LOOP =====
            case 7: // Gate - go to gate
                buildGatePaths();
                intake.setPower(-1);
                follower.followPath(gateFirstPath, true);
                setPathState(8);
                break;

            case 8: // Gate - wait at gate position
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(99);
                }
                break;

            case 99: // Gate - move back slightly
                turret.setDegreesTarget(63);
                turret.setPid();
                LL.set_angle_custom(0.17);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;

            case 102: // Gate - wait at back position

                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect artifacts
                double waitTime2 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                buildGatePathBack(waitTime2);


                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    ballCount = 3;
                    LL.set_angle_far_auto();
                    depo.setTargetVelocity(depo.farVeloblueauto);
                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10: // Gate - return to shooting position
                depo.updatePID();
                intake.setPower(1);

                if (!follower.isBusy()) {

                    actionTimer.resetTimer();
                    setPathState(110);
                }
                break;

            case 110: // Settle before gate shot
                depo.updatePID();
                intake.setPower(0);
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(11);
                }
                break;

            case 11: // Wait for shooting to complete
                if (actionState == 0) {
                    ballCount = 0;
                    gateHitCount++;

                    if (gateHitCount < TOTAL_GATE_CYCLES) {
                        setPathState(7);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            // ===== THIRD LINE PICKUP =====
            case 12: // Drive straight to third line pickup
                LL.set_angle_far_auto();
                depo.setTargetVelocity(depo.farVeloblueauto);
                intake.setPower(-1);
                follower.followPath(ThirdLinePickupPath, true);
                setPathState(13);
                break;

            case 13: // Wait until pickup reached
                depo.updatePID();
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    ballCount = 3;
                    setPathState(14);
                }
                break;

            case 14: // Drive straight back to shooting pose
                LL.set_angle_far_auto();
                depo.setTargetVelocity(depo.farVeloblueauto);

                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(15);
                break;

            case 15: // Wait until back at shooting pose
                depo.updatePID();

                if (!follower.isBusy()) {
                    intake.setPower(1);
                    actionTimer.resetTimer();
                    setPathState(115);
                }
                break;

            case 115: // Settle before final shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(16);
                }
                break;

            case 16: // Final shooting sequence
                intake.setPower(0);
                if (actionState == 0) {
                    buildGetOutPath();
                    setPathState(17);
                }
                break;

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
                LL.set_angle_far_auto();
                depo.setTargetVelocity(depo.farVeloblueauto);

                if (pathState != 2 && pathState != 6 && pathState != 11 && pathState != 16 &&
                        pathState != 101 && pathState != 105 && pathState != 110 && pathState != 115) {
                    break;
                }

                if (depo.reachedTargetHighTolerance()) {
                    greenInSlot = getGreenPos();
                    shootTimer.resetTimer();
                    setActionState(3);
                } else {
                    setActionState(2);
                }
                break;

            case 2: // Wait for shooter to spin up
                depo.updatePID();

                if (pathState != 2 && pathState != 6 && pathState != 11 && pathState != 16 &&
                        pathState != 101 && pathState != 105 && pathState != 110 && pathState != 115) {
                    setActionState(0); // Abort if somehow path state changed
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

                // ✅ ALWAYS use motif-based shooting from the start
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
        // ✅ Use motif from the very first shot
        if (motif.equals("gpp")) {
            // Green-Purple-Purple → shoot green first
            if (greenInSlot == 0) shootLRB(); // Green is left
            else if (greenInSlot == 1) shootRBL(); // Green is right
            else shootBLR(); // Green is back
        } else if (motif.equals("pgp")) {
            // Purple-Green-Purple → shoot purple first
            if (greenInSlot == 0) shootBLR(); // Green is left, so skip it first
            else if (greenInSlot == 1) shootLRB(); // Green is right, so skip it first
            else shootRBL(); // Green is back, so skip it first
        } else { // ppg or default
            // Purple-Purple-Green → shoot purples first
            if (greenInSlot == 0) shootRBL(); // Green is left, shoot right then back
            else if (greenInSlot == 1) shootBLR(); // Green is right, shoot back then left
            else shootLRB(); // Green is back, shoot left then right
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

    private int getGreenPos() {
        int pos = LL.sensors.getLeft();
        if (pos == 1) return 0;
        pos = LL.sensors.getRight();
        if (pos == 1) return 2;
        return 1;
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
                .addPath(new Path(new BezierCurve(cur, midpoint1, secondLinePickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), secondLinePickupPose.getHeading(), 0.8)
                .build();

        bezierSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(secondLinePickupPose, midpoint1, farshotpose)))
                .setLinearHeadingInterpolation(secondLinePickupPose.getHeading(), farshotpose.getHeading(), 0.8)
                .setTimeoutConstraint(0.1)
                .build();

        ThirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(midpoint2, ThirdPickupPose)))
                .setLinearHeadingInterpolation(midpoint2.getHeading(), ThirdPickupPose.getHeading())
                .build();
    }

    private void buildGatePaths() {
        Pose cur = follower.getPose();
        gateFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint3, lever_touch)))
                .setLinearHeadingInterpolation(cur.getHeading(), lever_touch.getHeading(),0.65)
                .setTimeoutConstraint(1)
                .build();

        gatebackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(lever_touch, back_lever)))
                .setLinearHeadingInterpolation(back_lever.getHeading(), back_lever.getHeading(), 0.1)
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

    private void buildGetOutPath() {
        Pose cur = follower.getPose();
        getOut = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, outPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), outPose.getHeading())
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

    // ========== UTILITY METHODS ==========

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