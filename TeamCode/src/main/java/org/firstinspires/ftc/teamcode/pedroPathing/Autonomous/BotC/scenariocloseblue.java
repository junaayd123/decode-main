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

@Autonomous(name = "scenariocloseblue", group = "Pedro")
public class scenariocloseblue extends OpMode {

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
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer, intakeWaitTimer;

    // ========== SCENARIO ENUM ==========
    public enum Scenario {
        SCENARIO_1_NO_ALLIANCE(1, 1, 1.6, 1.2),      // No alliance auto - 15 balls (1 gate cycle, has third line)
        SCENARIO_2_3_BALL_ALLIANCE(1, 1, 1.6, 1.2),  // Alliance 3 ball - 18 balls (1 gate cycle, has third line)
        SCENARIO_3_6_BALL_ALLIANCE(2, 0, 1.6, 1.2),  // Alliance 6 ball - 21 balls (2 gate cycles, no third line)
        SCENARIO_4_9_BALL_ALLIANCE(2, 0, 2.0, 1.4),  // Alliance 9 ball - 21 balls (2 gate cycles, custom wait)
        SCENARIO_5_EXCESS_CYCLE(2, 0, 2.0, 1.4),     // Alliance cycles excess - 24 balls (2 gate cycles, custom wait)
        SCENARIO_6_THREE_LINES(0, 0, 1.6, 1.2);      // ✅ NEW: First → Second → Third line sequence (12 balls, no gates)

        public final int gateCycles;           // Number of gate cycles to do
        public final int thirdLinePickup;      // 0 = skip, 1 = do third line pickup
        public final double gateWaitFirst;     // Wait time for first gate cycle
        public final double gateWaitLater;     // Wait time for later gate cycles

        Scenario(int gateCycles, int thirdLinePickup, double gateWaitFirst, double gateWaitLater) {
            this.gateCycles = gateCycles;
            this.thirdLinePickup = thirdLinePickup;
            this.gateWaitFirst = gateWaitFirst;
            this.gateWaitLater = gateWaitLater;
        }
    }

    // ========== SCENARIO SELECTION ==========
    private Scenario currentScenario = Scenario.SCENARIO_3_6_BALL_ALLIANCE;  // Default to Scenario 3

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int shooterSequence;
    private int greenInSlot;
    private String motif = "ppg"; // ✅ Default motif
    private String detectedMotif = ""; // ✅ Track detected motif
    private int gateHitCount = 0;
    private int shotCycleCount = 0;
    private boolean intakeRunning = false;
    private boolean thirdLineDone = false;
    private int ballCount = 3;  // ✅ Track number of balls (start with 3 preloaded)
    private boolean waitingForMoreBalls = false;
    private boolean secondLineDone = false; // ✅ Track if second line is done for Scenario 6

    // ========== CONSTANTS ==========
    private static final double SHOOT_INTERVAL = 0.335;
    private static final double SECOND_HOP_IN = 8;
    private static final double SETTLE_TIME = 0.3;
    private static final double REVERSE_TIME = 0.15;
    private static final double EXTRA_INTAKE_TIME = 0.5;

    // ========== POSES ==========
    private final Pose startPose = new Pose(44, -128, Math.toRadians(-35));
    private final Pose nearshotpose = new Pose(12, -81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, -81.5, Math.toRadians(-34));
    private final Pose firstPickupPose = new Pose(67, -81, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13.4, -58, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(10, -68, Math.toRadians(0));
    private final Pose secondpickuppose = new Pose(58
            , -55, Math.toRadians(0));

    private final Pose midpointopengate = new Pose(13.4, -68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, -60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(57.2, -56.1, Math.toRadians(-34));
    private final Pose back_lever = new Pose(58.3, -50.3, Math.toRadians(-36.5));
    private final Pose outfromgate = new Pose(50, -50, Math.toRadians(-42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, -58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, -53, Math.toRadians(-40));
    private final Pose intake_from_gate_rotate = new Pose(55, -54, Math.toRadians(0));
    private final Pose thirdLinePickupPose = new Pose(60, -33, Math.toRadians(0));
    private final Pose midpoint5 = new Pose(10, -30, Math.toRadians(0)); // ✅ Scenario 6: Before third line pickup
    private final Pose outPose = new Pose(26, -81.5, Math.toRadians(-34));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain firstLinePickupPath;
    private PathChain firstLineSecondHopPath;
    private PathChain thirdLinePickupPath;
    private PathChain thirdLineReturnPath;
    private PathChain gatebackPath;
    private PathChain getOut;

    // ========== CONTROLLER STATE TRACKING ==========
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;
    private boolean prevCircle = false;
    private boolean prevTriangle = false; // ✅ Added for Scenario 6

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        intakeWaitTimer = new Timer();

        depo = new Deposition_C(hardwareMap);
        LL = new lifters(hardwareMap);
        sensors = new ColorSensors(hardwareMap);
        turret = new TurretLimelight(hardwareMap);
        turret.setBlueAlliance();

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
        turret.setDegreesTarget(150);

        initAprilTag();

        telemetry.addLine("State-based Auto initialized - WITH SCENARIO 6");
        telemetry.addData("Scenario", currentScenario.name());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();
        detectMotifFromAprilTags();

        boolean currDpadDown = gamepad1.dpad_down;
        boolean currDpadLeft = gamepad1.dpad_left;
        boolean currDpadRight = gamepad1.dpad_right;
        boolean currDpadUp = gamepad1.dpad_up;
        boolean currCircle = gamepad1.circle;
        boolean currTriangle = gamepad1.triangle;

        if (currDpadDown && !prevDpadDown) {
            currentScenario = Scenario.SCENARIO_1_NO_ALLIANCE;
        } else if (currDpadLeft && !prevDpadLeft) {
            currentScenario = Scenario.SCENARIO_2_3_BALL_ALLIANCE;
        } else if (currDpadRight && !prevDpadRight) {
            currentScenario = Scenario.SCENARIO_4_9_BALL_ALLIANCE;
        } else if (currDpadUp && !prevDpadUp) {
            currentScenario = Scenario.SCENARIO_5_EXCESS_CYCLE;
        } else if (currCircle && !prevCircle) {
            currentScenario = Scenario.SCENARIO_3_6_BALL_ALLIANCE;
        } else if (currTriangle && !prevTriangle) {
            currentScenario = Scenario.SCENARIO_6_THREE_LINES;
            telemetry.addLine("✅ Scenario 6: First→Second→Third line sequence (12 balls)");
        }

        prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft;
        prevDpadRight = currDpadRight;
        prevDpadUp = currDpadUp;
        prevCircle = currCircle;
        prevTriangle = currTriangle;

        telemetry.addLine("D=1 | L=2 | R=4 | U=5 | Circle=3 | Triangle=6 (NEW)");
        telemetry.addData("Detected Motif", detectedMotif.isEmpty() ? "NONE" : detectedMotif);
        telemetry.addData("Will Use", motif);
        telemetry.addData("Scenario", currentScenario.name());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(46);
        turret.setPid();
        shotCycleCount = 0;
        gateHitCount = 0;
        thirdLineDone = false;
        secondLineDone = false;
        ballCount = 3; // ✅ Start with 3 preloaded
        waitingForMoreBalls = false;

        if (detectedMotif.isEmpty()) {
            telemetry.addLine("⚠ No AprilTag! Using default: " + motif);
        } else {
            telemetry.addLine("✓ Locked motif: " + motif);
        }
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

        telemetry.addData("Path State", pathState);
        telemetry.addData("Action State", actionState);
        telemetry.addData("Shot Cycle", shotCycleCount);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Scenario", currentScenario.name());

        if (pathState == -1) {
            telemetry.addData("Auto Status", "Complete");
            return;
        }

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

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
                    if (detection.id == 21) { motif = "gpp"; detectedMotif = "gpp"; }
                    if (detection.id == 22) { motif = "pgp"; detectedMotif = "pgp"; }
                    if (detection.id == 23) { motif = "ppg"; detectedMotif = "ppg"; }
                } else if (yaw > -90 && yaw < -40) {
                    if (detection.id == 21) { motif = "ppg"; detectedMotif = "ppg"; }
                    if (detection.id == 22) { motif = "gpp"; detectedMotif = "gpp"; }
                    if (detection.id == 23) { motif = "pgp"; detectedMotif = "pgp"; }
                }
            }
        }
    }

    // ========== PATH STATE MACHINE ==========
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Go back to near shot pose - START FLYWHEEL
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);
                buildGoBackPath();
                follower.followPath(goBackPath, true);
                setPathState(1);
                break;

            case 1: // Wait to reach near shot pose
                depo.updatePID();
                if (!follower.isBusy()) {
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

            case 2: // Wait for shooting to complete (preload)
                if (actionState == 0) {
                    ballCount = 0; // ✅ Shot 3 balls
                    turret.setDegreesTarget(7);

                    // ✅ Scenario 6: Go to FIRST line instead of second
                    if (currentScenario == Scenario.SCENARIO_6_THREE_LINES) {
                        setPathState(12); // Go to first line pickup
                    } else {
                        setPathState(3); // Normal flow: second line
                    }
                }
                break;

            case 3: // Bezier curve pickup - first path (second line)
                buildBezierPaths();
                intake.setPower(-1);
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4: // Wait for first bezier path
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    // ✅ Count balls
                    int sensedBalls = 0;
                    if (sensors.getLeft() != 0) sensedBalls++;
                    if (sensors.getRight() != 0) sensedBalls++;
                    if (sensors.getBack() != 0) sensedBalls++;
                    ballCount = sensedBalls;

                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);

                    // ✅ Only out-take if 3+ balls
                    if (ballCount >= 3) {
                        intake.setPower(1);
                    } else {
                        intake.setPower(0);
                    }

                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for second bezier path
                depo.updatePID();
                if (ballCount >= 3) {
                    intake.setPower(1);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    actionTimer.resetTimer();
                    setPathState(105);
                }
                break;

            case 105: // Settle before second shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooting cycle 2
                if (actionState == 0) {
                    ballCount = 0; // ✅ Shot 3 balls
                    gateHitCount = 0;
                    if (currentScenario.gateCycles > 0) {
                        setPathState(7);
                    } else if (currentScenario == Scenario.SCENARIO_6_THREE_LINES && !thirdLineDone) {
                        // ✅ Scenario 6: After second line, go to THIRD line
                        setPathState(20);
                    } else if (currentScenario.thirdLinePickup == 1 && !thirdLineDone) {
                        setPathState(20);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            // ===== GATE CYCLE LOOP =====
            case 7: case 8: case 99: case 102: case 9: case 10: case 110: case 11:
                handleGateCycle();
                break;

            // ===== THIRD LINE PICKUP (States 20-25) =====
            case 20: // Drive to third line
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);
                intake.setPower(-1);
                buildThirdLinePickupPath();
                follower.followPath(thirdLinePickupPath, true);
                setPathState(21);
                break;

            case 21: // Wait at third line
                intake.setPower(-1);
                depo.updatePID();
                if (!follower.isBusy()) {
                    // ✅ Count balls
                    int sensedBalls = 0;
                    if (sensors.getLeft() != 0) sensedBalls++;
                    if (sensors.getRight() != 0) sensedBalls++;
                    if (sensors.getBack() != 0) sensedBalls++;
                    ballCount = sensedBalls;
                    setPathState(22);
                }
                break;

            case 22: // Return from third line
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                // ✅ Only out-take if 3+ balls
                if (ballCount >= 3) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                buildThirdLineReturnPath();
                follower.followPath(thirdLineReturnPath, true);
                setPathState(23);
                break;

            case 23: // Wait to return
                depo.updatePID();
                if (ballCount >= 3) {
                    intake.setPower(1);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
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

            case 25: // Wait for third line shot
                if (actionState == 0) {
                    thirdLineDone = true;
                    ballCount = 0; // ✅ Shot 3 balls

                    // ✅ Scenario 6: After third line, go straight to get out
                    if (currentScenario == Scenario.SCENARIO_6_THREE_LINES) {
                        buildGetOutPath();
                        setPathState(18);
                    } else {
                        setPathState(12);
                    }
                }
                break;

            // ===== FIRST LINE PICKUP (States 12-17) =====
            case 12: // Drive to first line
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);
                ballCount = 0;
                waitingForMoreBalls = false;
                intake.setPower(-1);  // ✅ LINE 497: Intake turns ON (collecting)
                intakeRunning = true;
                buildLinePickupPaths();
                follower.followPath(firstLinePickupPath, true);
                setPathState(13);
                break;

            case 13: // Collect balls at first line
                depo.updatePID();
                int currentBalls = 0;
                if (sensors.getRight() != 0) currentBalls++;
                if (sensors.getBack() != 0) currentBalls++;
                if (sensors.getLeft() != 0) currentBalls++;

                if (currentBalls > ballCount) ballCount = currentBalls;

                if (ballCount >= 3 && !waitingForMoreBalls) {
                    waitingForMoreBalls = true;
                    intakeWaitTimer.resetTimer();
                }

                boolean timeToStop = waitingForMoreBalls &&
                        intakeWaitTimer.getElapsedTimeSeconds() >= EXTRA_INTAKE_TIME;

                if (timeToStop && ballCount >= 3) {
                    intake.setPower(-1);  // ✅ LINE 522: Intake turns OFF after collecting
                    intakeRunning = false;
                    actionTimer.resetTimer();
                    setPathState(14);
                }
                break;

            case 14: // Quick reverse
                if (actionTimer.getElapsedTimeSeconds() < REVERSE_TIME) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                    setPathState(15);
                }
                break;

            case 15: // Return from first line
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                // ✅ Only out-take if 3+ balls
                if (ballCount >= 3) {
                    intake.setPower(1);  // ✅ LINE 545: Out-take turns ON (only if 3+ balls)
                } else {
                    intake.setPower(0);
                }

                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(16);
                break;

            case 16: // Wait to return
                depo.updatePID();
                if (ballCount >= 3) {
                    intake.setPower(-1);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    actionTimer.resetTimer();
                    setPathState(115);
                }
                break;

            case 115: // Settle before first line shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(17);
                }
                break;

            case 17: // Wait for first line shot
                if (actionState == 0) {
                    ballCount = 0; // ✅ Shot 3 balls

                    // ✅ Scenario 6: After first line, go to SECOND line
                    if (currentScenario == Scenario.SCENARIO_6_THREE_LINES && !secondLineDone) {
                        secondLineDone = true;
                        setPathState(3); // Go to second line (bezier paths)
                    } else {
                        buildGetOutPath();
                        setPathState(18);
                    }
                }
                break;

            case 18:
                follower.followPath(getOut, true);
                setPathState(19);
                break;

            case 19:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    private void handleGateCycle() {
        switch (pathState) {
            case 7:
                double waitTime = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;
                buildGatePaths(waitTime);
                intake.setPower(-1);
                follower.followPath(gateFirstPath, true);
                setPathState(8);
                break;
            case 8:
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(99);
                }
                break;
            case 99:
                intake.setPower(-1);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;
            case 102:
                intake.setPower(-1);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                double waitTime2 = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;
                intake.setPower(-1);
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    // ✅ Count balls
                    int sensedBalls = 0;
                    if (sensors.getLeft() != 0) sensedBalls++;
                    if (sensors.getRight() != 0) sensedBalls++;
                    if (sensors.getBack() != 0) sensedBalls++;
                    ballCount = sensedBalls;

                    LL.set_angle_close();
                    depo.setTargetVelocity(depo.closeVelo_New_auto);
                    buildGatePathsBack();
                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;
            case 10:
                depo.updatePID();
                if (ballCount >= 3) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    actionTimer.resetTimer();
                    setPathState(110);
                }
                break;
            case 110:
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(11);
                }
                break;
            case 11:
                if (actionState == 0) {
                    ballCount = 0; // ✅ Shot 3 balls
                    gateHitCount++;
                    if (gateHitCount < currentScenario.gateCycles) {
                        setPathState(7);
                    } else {
                        if (currentScenario.thirdLinePickup == 1 && !thirdLineDone) {
                            setPathState(20);
                        } else {
                            setPathState(12);
                        }
                    }
                }
                break;
        }
    }

    // ========== ACTION STATE MACHINE ==========
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0: break;
            case 1:
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);
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
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) LL.leftUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.rightUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.backUp();
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) LL.backUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.leftUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.rightUp();
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL - 0.05) LL.rightUp();
        else if (t >= SHOOT_INTERVAL - 0.05 && t < SHOOT_INTERVAL) LL.allDown();
        else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2 - 0.05) LL.backUp();
        else if (t >= SHOOT_INTERVAL * 2 - 0.05 && t < SHOOT_INTERVAL * 2) LL.allDown();
        else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) LL.leftUp();
    }

    private int getGreenPos() {
        int pos = LL.sensors.getLeft();
        if (pos == 1) return 0;
        pos = LL.sensors.getRight();
        if (pos == 1) return 2;
        return 1;
    }

    // ========== PATH BUILDING ==========
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
                .setTimeoutConstraint(1)
                .build();
        gatebackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(infront_of_lever_new, back_lever)))
                .setLinearHeadingInterpolation(back_lever.getHeading(), back_lever.getHeading(), 0.1)
                .setTimeoutConstraint(0.3)
                .build();
    }

    private void buildGatePathsBack() {
        Pose cur = follower.getPose();
        gateSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, nearshotpose2)))
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

    private void buildThirdLinePickupPath() {
        Pose cur = follower.getPose();

        // ✅ Scenario 6: Use midpoint5 for smoother path from second line to third line
        if (currentScenario == Scenario.SCENARIO_6_THREE_LINES) {
            thirdLinePickupPath = follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(cur, midpoint5, thirdLinePickupPose)))
                    .setLinearHeadingInterpolation(cur.getHeading(), thirdLinePickupPose.getHeading(), 0.5)
                    //.setLinearHeadingInterpolation(secondpickuppose.getHeading(), nearshotpose2.getHeading(), 0.8)
                    .build();
        }
    }

    private void buildThirdLineReturnPath() {
        Pose cur = follower.getPose();
        thirdLineReturnPath = follower.pathBuilder()
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
        if (!intakeRunning && actionTimer.getElapsedTimeSeconds() < REVERSE_TIME && actionTimer.getElapsedTimeSeconds() > 0) {
            intake.setPower(1);
        } else if (!intakeRunning && actionTimer.getElapsedTimeSeconds() >= REVERSE_TIME) {
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
