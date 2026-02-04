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

@Autonomous(name = "scenariofarred", group = "Pedro")
public class scenariofarred extends OpMode {

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
    /** Used only in excess states 31/32 so timeout is from when we entered that state (pathTimer is reset by setPathState). */
    private Timer excessPathTimeoutTimer;
    private boolean thirdLineDone;


    // ========== SCENARIO ENUM ==========
    // FAR: 4 scenarios (1 and 2 merged - same sequence)
    public enum Scenario {
        // 1: No alliance OR 3 ball - 15 balls: preload → second line → 1 gate → first line → third line
        SCENARIO_1_NO_OR_3_BALL(1, 1, 0, false, 1.6, 1.2),
        // 2: Alliance 6 ball - 15 balls: preload → second line → 2 gates → first line
        SCENARIO_2_6_BALL(2, 0, 0, false, 1.6, 1.2),
        // 3: Alliance 9 ball - 12 balls: preload → third line → gate → excess
        SCENARIO_3_9_BALL(1, 0, 1, true, 2.0, 1.4),
        // 4: Alliance dominates - 12 balls: preload → third line → excess → excess
        SCENARIO_4_DOMINATES(0, 0, 2, true, 2.0, 1.4),
        // 5: 12 balls - preload → third line → second line → first line → get out
        SCENARIO_5_9MOTIF(0, 0, 0, true, 1.6, 1.2);

        public final int gateCycles;
        public final int thirdLinePickup;      // 1 = do third line after first line (Scenario 1 only)
        public final int excessPickups;        // 0, 1, or 2 - number of excess area pickups
        public final boolean skipSecondLine;   // true = go to third line right after preload (Scenarios 3, 4)
        public final double gateWaitFirst;
        public final double gateWaitLater;

        Scenario(int gateCycles, int thirdLinePickup, int excessPickups, boolean skipSecondLine,
                 double gateWaitFirst, double gateWaitLater) {
            this.gateCycles = gateCycles;
            this.thirdLinePickup = thirdLinePickup;
            this.excessPickups = excessPickups;
            this.skipSecondLine = skipSecondLine;
            this.gateWaitFirst = gateWaitFirst;
            this.gateWaitLater = gateWaitLater;
        }
    }

    // ========== SCENARIO SELECTION ==========
    // 1: No/3 ball - 15 balls: preload, second line, 1 gate, first line, third line
    // 2: 6 ball - 15 balls: preload, second line, 2 gates, first line
    // 3: 9 ball - 12 balls: preload, third line, gate, excess
    // 4: Dominates - 12 balls: preload, third line, excess, excess
    private Scenario currentScenario = Scenario.SCENARIO_2_6_BALL;  // Default to Scenario 2

    /** Scenario 3 only: wait this many seconds (idle) after third line shoot before going to gate. Set in init. */
    private double scenario3WaitBeforeGateSeconds = 0;

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int shooterSequence;
    private int greenInSlot;
    private String motif = "empty";
    private int gateHitCount = 0;
    private int shotCycleCount = 0;  // Tracks how many 3-ball cycles completed
    private boolean intakeRunning = false;

    // ========== CONSTANTS ==========
    private static double SHOOT_INTERVAL = 0.335;
    private static final double SECOND_HOP_IN = 8;
    private static final double SETTLE_TIME = 0.3;  // Time to settle before shooting

    // ========== POSES ==========
    private final Pose startPose = new Pose(7+6.5, 7, Math.toRadians(0));
    private final Pose nearshotpose = new Pose(12, 81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, 81.5, Math.toRadians(34));
    private final Pose ThirdPickupPose = new Pose(59, 37, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13, 60, Math.toRadians(0));
    private final Pose farshotpose = new Pose(12, 17, Math.toRadians(0));
    private final Pose outPose = new Pose(30, 17, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(23, 35, Math.toRadians(0));
    private final Pose midpoint3 = new Pose(21, 61, Math.toRadians(0));
    private final Pose midpoint4 = new Pose(20,83, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(52, 84, Math.toRadians(0));
    private final Pose secondLinePickupPose = new Pose(59, 59, Math.toRadians(0));
    private final Pose secondpickupPose = new Pose(56, 38, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, 68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, 60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(62, 62, Math.toRadians(34));
    private final Pose back_lever = new Pose(63, 54, Math.toRadians(38));
    private final Pose infront_of_lever_adj = new Pose(60.5, 61, Math.toRadians(34));

    private final Pose outfromgate = new Pose(50, 50, Math.toRadians(42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, 58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, 53, Math.toRadians(40));
    private final Pose intake_from_gate_rotate = new Pose(55, 54, Math.toRadians(0));
    /** Excess ball area - first position, then we strafe in -y. */
    private final Pose excessBallArea = new Pose(61, 12, Math.toRadians(-10));
    /** End of excess strafe (slow strafe to the right, -y direction). */
    private final Pose excessBallAreaStrafeEnd = new Pose(60, 9.8, Math.toRadians(-13));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain getOut;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain ThirdLinePickupPath;
    private PathChain firstLinePickupPath;

    private PathChain firstLineSecondHopPath;
    private PathChain gatebackPath;
    private PathChain excessPath;
    /** Second motion: slow strafe in -y from excessBallArea to excessBallAreaStrafeEnd (1.8 s timeout). */
    private PathChain excessPathStrafe;

    /** Speed multiplier for excess area path (0.5 = half speed). */
    private static final double EXCESS_PATH_SPEED = 0.5;

    /** Count of excess pickups done this run (for scenarios 3 & 4). */
    private int excessPickupCount = 0;

    // ========== CONTROLLER STATE TRACKING ==========
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;
    private boolean prevCircle = false;
    private boolean prevTriangle = false;

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
        turret.setDegreesTarget(-100);

        // Initialize AprilTag vision
        initAprilTag();

        telemetry.addLine("State-based Auto initialized (Webcam) - OPTIMIZED");
        telemetry.addData("Scenario", currentScenario.name());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        // Detect motif from AprilTags using webcam
        detectMotifFromAprilTags();

        // Handle scenario selection with button press detection
        boolean currDpadDown = gamepad1.dpad_down;
        boolean currDpadLeft = gamepad1.dpad_left;
        boolean currDpadRight = gamepad1.dpad_right;
        boolean currDpadUp = gamepad1.dpad_up;
        boolean currCircle = gamepad1.circle;
        boolean currTriangle = gamepad1.triangle;

        // Scenario 3: adjust wait time before gate (L bumper = minus 0.5s, R bumper = plus 0.5s)
        if (currentScenario == Scenario.SCENARIO_3_9_BALL) {
            if (gamepad1.left_bumper) scenario3WaitBeforeGateSeconds = Math.max(0, scenario3WaitBeforeGateSeconds - 0.5);
            if (gamepad1.right_bumper) scenario3WaitBeforeGateSeconds += 0.5;
        }

        // Detect button presses (transition from not pressed to pressed)
        if (currDpadDown && !prevDpadDown) {
            currentScenario = Scenario.SCENARIO_1_NO_OR_3_BALL;
            telemetry.addLine("Scenario 1: alliance No/3 ball - 15 balls, 1 gate, first line, third line");
        } else if (currDpadLeft && !prevDpadLeft) {
            currentScenario = Scenario.SCENARIO_2_6_BALL;
            telemetry.addLine("Scenario 2: alliance 6 ball - 15 balls, 2 gates, first line only");
        } else if (currDpadRight && !prevDpadRight) {
            currentScenario = Scenario.SCENARIO_3_9_BALL;
            telemetry.addLine("Scenario 3: alliance 9 ball - 12 balls, third line, gate, excess");
        } else if (currDpadUp && !prevDpadUp) {
            currentScenario = Scenario.SCENARIO_4_DOMINATES;
            telemetry.addLine("Scenario 4: Alliance Dominates - we do 12 balls, third line, excess, excess");
        } else if (currTriangle && !prevTriangle) {
            currentScenario = Scenario.SCENARIO_5_9MOTIF;
            telemetry.addLine("Scenario 5: 3 lines pickup - 12 balls");
        } else if (currCircle && !prevCircle) {
            currentScenario = Scenario.SCENARIO_2_6_BALL;
            telemetry.addLine("Scenario 2: alliance 6 ball (default)");
        }

        // Update previous button states
        prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft;
        prevDpadRight = currDpadRight;
        prevDpadUp = currDpadUp;
        prevCircle = currCircle;
        prevTriangle = currTriangle;

        telemetry.addLine("Use D-Pad / Triangle / Circle to select scenario:");
        telemetry.addLine("DPad D=1 | L=2 | R=3 | U=4 | Triangle=5 (Third→Second→First) | Circle=2 (Default)");
        telemetry.addData("Motif Detected", motif);
        telemetry.addData("Scenario", currentScenario.name());
        telemetry.addData("Gate Cycles", currentScenario.gateCycles);
        telemetry.addData("Third Line", currentScenario.thirdLinePickup == 1 ? "Yes" : "No");
        telemetry.addData("Excess Pickups", currentScenario.excessPickups);
        if (currentScenario == Scenario.SCENARIO_3_9_BALL) {
            telemetry.addData("Scenario 3: Wait before gate (s)", scenario3WaitBeforeGateSeconds);
            telemetry.addLine("L/R Bumper: decrease/increase wait");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(-68.6);
        turret.setPid();
        shotCycleCount = 0;
        gateHitCount = 0;
        thirdLineDone = false;
        excessPickupCount = 0;
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
            telemetry.addData("Gate Cycle", (gateHitCount + 1) + "/" + currentScenario.gateCycles);
        } else if (pathState >= 12 && pathState <= 16) {
            telemetry.addData("Sequence", "First Line Pickup");
        } else if (pathState >= 20 && pathState <= 25) {
            telemetry.addData("Sequence", "Third Line Pickup");
        } else if (pathState >= 30 && pathState <= 36) {
            telemetry.addData("Sequence", "Excess Area Pickup");
        }
        telemetry.addData("Scenario", currentScenario.name());
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
            case 0: // Start - spin up flywheel
                // ✅ Start spinning flywheel at the very beginning
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto);
                SHOOT_INTERVAL = 0.375;
                setPathState(1);
                break;

            case 1: // Wait for flywheel to spin up
                depo.updatePID();  // ✅ Keep updating PID
                if (depo.reachedTargetHighTolerance()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(101);  // ✅ Go to settling state
                }
                break;

            case 101: // ✅ NEW STATE - Settle before first shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1); // Start shooting
                    setPathState(2);
                }
                break;

            case 2: // Wait for shooting to complete (preload)
                intake.setPower(-1);
                if (actionState == 0) { // Shooting done
                    SHOOT_INTERVAL = 0.335;
                    // Scenario 3 or 4: skip second line, go straight to third line
                    if (currentScenario.skipSecondLine) {
                        setPathState(20);
                    } else {
                        setPathState(3);
                    }
                }
                break;

            case 3: // Bezier curve pickup - first path
                buildBezierPaths();
                LL.set_angle_far_auto();
                intake.setPower(-1);  // ⭐ START INTAKE BEFORE GOING TO SECOND LINE
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4: // Wait for first bezier path
                depo.updatePID();
                if (isPathComplete()) {
                    LL.set_angle_far();
                    depo.setTargetVelocity(depo.farVeloredauto2);
                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for second bezier path
                depo.updatePID();  // ✅ Keep updating PID during drive
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

            case 6: // Wait for shooting cycle 2 (after second line shoot)
                intake.setPower(0);
                if (actionState == 0) {
                    gateHitCount = 0; // Reset counter
                    // Scenario 5 (third→second→first): gateCycles=0 → go to first line (12), then case 16 → get out
                    if (currentScenario.gateCycles > 0) {
                        setPathState(7); // Start gate cycles
                    } else {
                        setPathState(12); // First line pickup (or Scenario 5: first line then get out)
                    }
                }
                break;

            // ===== GATE CYCLE LOOP =====
            case 7: // Gate - go to gate
                double waitTime = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;
                buildGatePaths(waitTime);
                intake.setPower(-1);
                follower.followPath(gateFirstPath, true);
                setPathState(8);
                break;

            case 8: // Gate - wait at gate position
                depo.updatePID();
                if (isPathComplete()) {
                    actionTimer.resetTimer();
                    setPathState(99);
                }
                break;
            case 99: // Gate - go to back_lever
                double waitTime1 = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;

                intake.setPower(-1);
                follower.followPath(gatebackPath, true);
                setPathState(102);
                break;
            case 102: // Gate - wait at back_lever position
                if (isPathComplete()) {
                    buildGatePathBack(currentScenario.gateWaitFirst);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect artifacts
                double waitTime2 = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    // ✅ Start spinning flywheel BEFORE return path
                    LL.set_angle_far();
                    depo.setTargetVelocity(depo.farVeloredauto2);

                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10: // Gate - return to shooting position
                intake.setPower(1);
                depo.updatePID();  // ✅ Keep updating PID during drive
                if (isPathComplete()) {
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

                    if (gateHitCount < currentScenario.gateCycles) {
                        setPathState(7); // Loop back to gate cycle
                    } else {
                        // Scenario 3: gate done → excess; else → first line
                        if (currentScenario.excessPickups > 0) {
                            setPathState(30);
                        } else {
                            setPathState(12); // Gate cycles done → go to FIRST LINE pickup
                        }
                    }
                }
                break;

            // ===== FIRST LINE PICKUP (Scenarios 1 & 2: drive to audience line, collect, return to far shot, shoot) =====
            case 12: // Drive to first line (near audience - nearshotpose)
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                intake.setPower(-1);
                buildFirstLinePickupPath();
                follower.followPath(firstLinePickupPath, true);
                setPathState(13);
                break;

            case 13: // Wait until first line pickup reached
                depo.updatePID();
                if (isPathComplete()) {
                    setPathState(14);
                    manageSecondHopIntake();
                }
                break;

            case 14: // Drive back to far shooting pose from first line
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(15);
                break;

            case 15: // Wait until back at far shooting pose
                depo.updatePID();
                if (isPathComplete()) {
                    actionTimer.resetTimer();
                    setPathState(115);
                }
                break;

            case 115: // Settle before first line shot
                depo.updatePID();
                intake.setPower(1);
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(16);
                }
                break;

            case 16: // Wait for first line shooting to complete
                if (actionState == 0) {
                    intake.setPower(0);
                    // Scenarios 1 & 2: do third line pickup; else get out
                    if (currentScenario.thirdLinePickup == 1 && !thirdLineDone) {
                        setPathState(20); // Go to third line pickup
                    } else {
                        buildGetOutPath();
                        setPathState(17);
                    }
                }
                break;

            // ===== THIRD LINE PICKUP (Scenarios 1 & 2 only) =====
            case 20: // Drive to third line pickup
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                intake.setPower(-1);
                buildThirdLinePickupPath();
                follower.followPath(ThirdLinePickupPath, true);
                setPathState(21);
                break;

            case 21: // Wait until third line pickup reached
                depo.updatePID();
                if (isPathComplete()) {
                    setPathState(22);
                    manageSecondHopIntake();
                }
                break;

            case 22: // Drive back to far shooting pose from third line
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(23);
                break;

            case 23: // Wait until back at far shooting pose
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(24);
                }
                break;

            case 24: // Settle before third line shot
                depo.updatePID();
                intake.setPower(1);
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(25);
                }
                break;

            case 25: // Wait for third line shooting to complete
                if (actionState == 0) {
                    thirdLineDone = true;
                    intake.setPower(0);
                    // Scenario 5: third line → second line → first line → get out
                    if (currentScenario == Scenario.SCENARIO_5_9MOTIF) {
                        setPathState(3);   // Go to second line
                    } else if (currentScenario.excessPickups > 0) {
                        // Scenario 3: third line → [wait] → gate → excess. Scenario 4: third line → excess → excess.
                        if (currentScenario.gateCycles > 0) {
                            setPathState(26);  // Scenario 3: wait (or go straight to gate if 0)
                        } else {
                            setPathState(30);  // Go to excess (Scenario 4)
                        }
                    } else {
                        buildGetOutPath();
                        setPathState(17);
                    }
                }
                break;

            case 26: // Scenario 3 only: idle wait after third line shoot before going to gate (pathTimer reset on enter via setPathState(26))
                if (pathTimer.getElapsedTimeSeconds() >= scenario3WaitBeforeGateSeconds) {
                    gateHitCount = 0;
                    setPathState(7);
                }
                break;

            // ===== EXCESS AREA PICKUP (Scenarios 3 & 4, at 0.5 speed) =====
            case 30: // Drive to excess area at 0.5 speed
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                intake.setPower(-1);
                buildExcessPath();
                follower.setMaxPower(EXCESS_PATH_SPEED);  // 0.5 speed for excess area
                follower.followPath(excessPath, true);
                excessPathTimeoutTimer.resetTimer();
                setPathState(31);
                break;

            case 31: // Wait until first motion (61, 9, -10) reached, then start slow strafe in -y
                depo.updatePID();
                // Timeout: if path takes more than 1 second, skip it and continue
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    buildExcessStrafePath();
                    follower.followPath(excessPathStrafe, true);  // still at 0.5 speed
                    excessPathTimeoutTimer.resetTimer();
                    setPathState(32);
                }
                break;

            case 32: // Wait until strafe (1.8 s timeout) done
                depo.updatePID();
                // Timeout: if path takes more than 1 second, skip it and continue
                if (!follower.isBusy() || excessPathTimeoutTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.setMaxPower(1.0);
                    setPathState(33);
                    manageSecondHopIntake();
                }
                break;

            case 33: // Drive back to far shooting pose from excess
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto2);
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(34);
                break;

            case 34: // Wait until back at far shooting pose
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(35);
                }
                break;

            case 35: // Settle before excess shot
                depo.updatePID();
                intake.setPower(1);
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(36);
                }
                break;

            case 36: // Wait for excess shooting to complete
                if (actionState == 0) {
                    intake.setPower(0);
                    excessPickupCount++;
                    if (excessPickupCount < currentScenario.excessPickups) {
                        setPathState(30);  // Another excess pickup
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
                LL.set_angle_far();
                depo.setTargetVelocity(depo.farVeloredauto);

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
    private void buildGetOutPath(){
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

    /** First line = audience line (nearshotpose). Path from current pose (farshotpose after gates) to nearshotpose. */
    private void buildFirstLinePickupPath() {
        Pose cur = follower.getPose();
        firstLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint4, firstPickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), firstPickupPose.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    /** Third line = middle line (ThirdPickupPose). Path from current pose (farshotpose after first line) to ThirdPickupPose. */
    private void buildThirdLinePickupPath() {
        Pose cur = follower.getPose();
        ThirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint2, ThirdPickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), ThirdPickupPose.getHeading(), 0.8)
                .setTimeoutConstraint(0.1)
                .build();
    }

    /** Builds gateFirstPath (to lever) and gatebackPath (to back_lever). Both used in gate cycle; gateSecondPath built in 102. */
    private void buildGatePaths(double waitTime) {
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
                .setTimeoutConstraint(0.15)
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

    /** First motion: current pose to (61, 9, heading -10). Follow at EXCESS_PATH_SPEED (0.5). */
    private void buildExcessPath() {
        Pose cur = follower.getPose();
        excessPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, excessBallArea)))
                .setLinearHeadingInterpolation(cur.getHeading(), excessBallArea.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
    }

    /** Second motion: slow strafe from (61, 9, -10) in -y direction to excessBallAreaStrafeEnd. Timeout 1.8 s. */
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

    /**
     * Check if path is complete (follower says it's done).
     */
    private boolean isPathComplete() {
        return !follower.isBusy();
    }

    @Override
    public void stop() {
        stopShooter();
        if (intake != null) intake.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }
}
