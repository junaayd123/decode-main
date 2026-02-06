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
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    // ========== SCENARIO ENUM ==========
    public enum Scenario {
        SCENARIO_1_NO_ALLIANCE(1, 1, 1.6, 1.2),      // No alliance auto - 15 balls (1 gate cycle, has third line)
        SCENARIO_2_3_BALL_ALLIANCE(1, 1, 1.6, 1.2),  // Alliance 3 ball - 18 balls (1 gate cycle, has third line)
        SCENARIO_3_6_BALL_ALLIANCE(2, 0, 1.6, 1.2),  // Alliance 6 ball - 21 balls (2 gate cycles, no third line) (CURRENT)
        SCENARIO_4_9_BALL_ALLIANCE(2, 0, 2.0, 1.4),  // Alliance 9 ball - 21 balls (2 gate cycles, custom wait)
        SCENARIO_5_EXCESS_CYCLE(2, 0, 2.0, 1.4);     // Alliance cycles excess - 24 balls (2 gate cycles, custom wait)

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
    // Change this to select which scenario to run:
    // SCENARIO_1_NO_ALLIANCE: No alliance auto (15 balls) - 1 gate cycle, has third line pickup
    // SCENARIO_2_3_BALL_ALLIANCE: Alliance 3 ball auto (18 balls) - 1 gate cycle, has third line pickup
    // SCENARIO_3_6_BALL_ALLIANCE: Alliance 6 ball auto (21 balls) - 2 gate cycles, no third line (CURRENT)
    // SCENARIO_4_9_BALL_ALLIANCE: Alliance 9 ball auto (21 balls) - 2 gate cycles, custom wait times
    // SCENARIO_5_EXCESS_CYCLE: Alliance cycles excess (24 balls) - 2 gate cycles, custom wait times
    private Scenario currentScenario = Scenario.SCENARIO_3_6_BALL_ALLIANCE;  // Default to Scenario 3

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int shooterSequence;
    private int greenInSlot;
    private String motif = "empty";
    private int gateHitCount = 0;
    private int shotCycleCount = 0;  // Tracks how many 3-ball cycles completed
    private boolean intakeRunning = false;
    private boolean thirdLineDone = false;  // Track if third line pickup completed

    // ========== CONSTANTS ==========
    private static final double SHOOT_INTERVAL = 0.335;
    private static final double SECOND_HOP_IN = 8;
    private static final double SETTLE_TIME = 0.3;  // Time to settle before shooting

    // ========== POSES - CORRECTLY MIRRORED FROM CLOSE RED ==========
    // Close Red: X positive, Y positive, heading positive
    // Close Blue: X positive, Y negative, heading negative
    private final Pose startPose = new Pose(44, -128, Math.toRadians(-35));
    private final Pose nearshotpose = new Pose(12, -81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, -81.5, Math.toRadians(-34));
    private final Pose firstPickupPose = new Pose(42, -81, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13.4, -58, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(10, -68, Math.toRadians(0));
    private final Pose secondpickuppose = new Pose(56, -55, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, -68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, -60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(57.2, -56.1, Math.toRadians(-34));
    private final Pose back_lever = new Pose(58.3, -52.3, Math.toRadians(-36.5));
    private final Pose outfromgate = new Pose(50, -50, Math.toRadians(-42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, -58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, -53, Math.toRadians(-40));
    private final Pose intake_from_gate_rotate = new Pose(55, -54, Math.toRadians(0));
    private final Pose thirdLinePickupPose = new Pose(56, -45, Math.toRadians(0));  // Third line pickup position
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
        turret.setBlueAlliance();

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
        turret.setDegreesTarget(96.4);

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
        // Use gamepad1 for scenario selection (or gamepad2 if preferred)
        boolean currDpadDown = gamepad1.dpad_down;
        boolean currDpadLeft = gamepad1.dpad_left;
        boolean currDpadRight = gamepad1.dpad_right;
        boolean currDpadUp = gamepad1.dpad_up;
        boolean currCircle = gamepad1.circle;

        // Detect button presses (transition from not pressed to pressed)
        if (currDpadDown && !prevDpadDown) {
            currentScenario = Scenario.SCENARIO_1_NO_ALLIANCE;
            telemetry.addLine("Scenario 1 Selected: " +
                    "No alliance auto (15 balls) - 1 gate cycle, has third line pickup");
        } else if (currDpadLeft && !prevDpadLeft) {
            currentScenario = Scenario.SCENARIO_2_3_BALL_ALLIANCE;
            telemetry.addLine("Scenario 2 Selected: " +
                    "Alliance 3 ball auto (18 balls) - 1 gate cycle, has third line pickup");
        } else if (currDpadRight && !prevDpadRight) {
            currentScenario = Scenario.SCENARIO_4_9_BALL_ALLIANCE;
            telemetry.addLine("Scenario 4 Selected: " +
                    "Alliance 9 ball auto (21 balls) - 2 gate cycles, custom wait times");
        } else if (currDpadUp && !prevDpadUp) {
            currentScenario = Scenario.SCENARIO_5_EXCESS_CYCLE;
            telemetry.addLine("Scenario 5 Selected: " +
                    "Alliance cycles excess (24 balls) - 2 gate cycles, custom wait times");
        } else if (currCircle && !prevCircle) {
            currentScenario = Scenario.SCENARIO_3_6_BALL_ALLIANCE;
            telemetry.addLine("Scenario 3 Selected: " +
                    "Alliance 6 ball auto (21 balls) - 2 gate cycles, no third line pickup");
        }

        // Update previous button states
        prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft;
        prevDpadRight = currDpadRight;
        prevDpadUp = currDpadUp;
        prevCircle = currCircle;

        telemetry.addLine("Use D-Pad or Circle to select scenario:");
        telemetry.addLine("DPad Down = Scenario 1 | DPad Left = Scenario 2");
        telemetry.addLine("DPad Right = Scenario 4 | DPad Up = Scenario 5");
        telemetry.addLine("Circle = Scenario 3 (Default)");
        telemetry.addData("Motif Detected", motif);
        telemetry.addData("Scenario", currentScenario.name());
        telemetry.addData("Gate Cycles", currentScenario.gateCycles);
        telemetry.addData("Third Line Pickup", currentScenario.thirdLinePickup == 1 ? "Yes" : "No");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(44.5);
        turret.setPid();
        shotCycleCount = 0;
        gateHitCount = 0;
        thirdLineDone = false;
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
        } else if (pathState >= 12 && pathState <= 17) {
            if (pathState == 13 || pathState == 14) {
                telemetry.addData("Sequence", "First Line - Second Hop");
            } else {
                telemetry.addData("Sequence", "First Line Pickup");
            }
        } else if (pathState >= 20 && pathState <= 25) {
            telemetry.addData("Sequence", "Third Line Pickup");
        }
        telemetry.addData("Scenario", currentScenario.name());
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

                // Using blue side logic
                if (yaw > 40 && yaw < 90) {
                    if (detection.id == 21) motif = "gpp";
                    if (detection.id == 22) motif = "pgp";
                    if (detection.id == 23) motif = "ppg";
                } else if (yaw > -90 && yaw < -40) {
                    if (detection.id == 21) motif = "ppg";
                    if (detection.id == 22) motif = "gpp";
                    if (detection.id == 23) motif = "pgp";
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
                    turret.setDegreesTarget(7);
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
                    // Check if scenario requires gate cycles
                    if (currentScenario.gateCycles > 0) {
                        setPathState(7); // Start gate cycles
                    } else {
                        // Skip gate cycles, go to third line or first line
                        if (currentScenario.thirdLinePickup == 1 && !thirdLineDone) {
                            setPathState(20); // Go to third line pickup
                        } else {
                            setPathState(12); // Go to first line pickup
                        }
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
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect artifacts
                double waitTime2 = (gateHitCount == 0) ? currentScenario.gateWaitFirst : currentScenario.gateWaitLater;
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

                    if (gateHitCount < currentScenario.gateCycles) {
                        setPathState(7); // Loop back to gate cycle
                    } else {
                        // Gate cycles done, check if we need third line pickup
                        if (currentScenario.thirdLinePickup == 1 && !thirdLineDone) {
                            setPathState(20); // Go to third line pickup
                        } else {
                            setPathState(12); // Move to first line pickup
                        }
                    }
                }
                break;

            // ===== THIRD LINE PICKUP (Scenarios 1 & 2) =====
            case 20: // Drive to third line pickup
                // ✅ Start spinning flywheel BEFORE going to pickup
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                manageSecondHopIntake();
                buildThirdLinePickupPath();
                follower.followPath(thirdLinePickupPath, true);
                setPathState(21);
                break;

            case 21: // Wait until third line pickup reached
                depo.updatePID();  // ✅ Keep updating PID during drive
                if (!follower.isBusy()) {
                    manageSecondHopIntake();
                    setPathState(22);
                }
                break;

            case 22: // Drive back to shooting pose from third line
                // ✅ Start spinning flywheel BEFORE return path
                LL.set_angle_close();
                depo.setTargetVelocity(depo.closeVelo_New_auto);

                buildThirdLineReturnPath();
                follower.followPath(thirdLineReturnPath, true);
                setPathState(23);
                break;

            case 23: // Wait until back at shooting pose from third line
                depo.updatePID();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();  // ✅ Start settle timer
                    setPathState(24);  // ✅ Go to settling state
                }
                break;

            case 24: // ✅ NEW STATE - Settle before third line shot
                depo.updatePID();
                if (actionTimer.getElapsedTimeSeconds() > SETTLE_TIME) {
                    setActionState(1);
                    setPathState(25);
                }
                break;

            case 25: // Wait for third line shooting to complete
                if (actionState == 0) {
                    thirdLineDone = true;
                    setPathState(12); // Move to first line pickup
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
                    intake.setPower(1);
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
                .setTimeoutConstraint(1.6)
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
        thirdLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, thirdLinePickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), thirdLinePickupPose.getHeading())
                .build();
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
