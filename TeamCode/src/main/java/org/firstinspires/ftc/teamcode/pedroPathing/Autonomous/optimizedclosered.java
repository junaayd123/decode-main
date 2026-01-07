package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;

import java.util.List;

@Autonomous(name = "C-Bot Close Red (State)", group = "Pedro")
public class optimizedclosered extends OpMode {

    // ========== SUBSYSTEMS ==========
    private Follower follower;
    private Deposition_C depo;
    private TurretLimelight turret;
    private lifters LL;
    private ColorSensors sensors;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;

    // ========== TIMERS ==========
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;

    // ========== STATE VARIABLES ==========
    private int pathState;
    private int actionState;
    private int shooterSequence;
    private int greenInSlot;
    private String motif = "empty";
    private int gateHitCount = 0; // Track how many gate cycles we've done

    // ========== CONSTANTS ==========
    private static final double SHOOT_INTERVAL = 0.40;
    private static final double SECOND_HOP_IN = 8;
    private static final double GATE_WAIT_TIME_FIRST = 1.6;
    private static final double GATE_WAIT_TIME_LATER = 1.2;
    private static final int TOTAL_GATE_CYCLES = 2; // Only 2 gate cycles now

    // ========== POSES ==========
    private final Pose startPose = new Pose(44, 128, Math.toRadians(35));
    private final Pose nearshotpose = new Pose(12, 81.5, Math.toRadians(0));
    private final Pose nearshotpose2 = new Pose(12, 81.5, Math.toRadians(34));
    private final Pose firstPickupPose = new Pose(42, 81, Math.toRadians(0));
    private final Pose midpoint1 = new Pose(13.4, 58, Math.toRadians(0));
    private final Pose midpoint2 = new Pose(10, 68, Math.toRadians(0));
    private final Pose firstpickupPose = new Pose(56, 55, Math.toRadians(0));
    private final Pose midpointopengate = new Pose(13.4, 68, Math.toRadians(0));
    private final Pose infront_of_lever = new Pose(54, 60, Math.toRadians(0));
    private final Pose infront_of_lever_new = new Pose(57.2, 56.1, Math.toRadians(34)); //y changed from 57 to 56 on 1/5/2026 - vihaan
    private final Pose outfromgate = new Pose(50, 50, Math.toRadians(42));
    private final Pose midpointbefore_intake_from_gate = new Pose(52, 58, Math.toRadians(0));
    private final Pose intake_from_gate = new Pose(56, 53, Math.toRadians(40));
    private final Pose intake_from_gate_rotate = new Pose(55, 54, Math.toRadians(0));

    // ========== PATHS ==========
    private PathChain goBackPath;
    private PathChain bezierFirstPath;
    private PathChain bezierSecondPath;
    private PathChain gateFirstPath;
    private PathChain gateSecondPath;
    private PathChain firstLinePickupPath;
    private PathChain firstLineSecondHopPath;

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
        turret.limelight.pipelineSwitch(2);
        turret.limelight.start();
        turret.resetTurretEncoder();
        turret.setDegreesTarget(-96.4);

        telemetry.addLine("State-based Auto initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        turret.setPid();
        turret.toTargetInDegrees();

        // Detect motif from AprilTags
        LLResult result = turret.limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            if (fr.getFiducialId() == 21) motif = "pgp";
            if (fr.getFiducialId() == 22) motif = "ppg";
            if (fr.getFiducialId() == 23) motif = "gpp";
        }

        telemetry.addData("Motif Detected", motif);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.setDegreesTarget(-44.5);
        turret.setPid();
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

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Motif", motif);
        telemetry.update();
    }

    // ========== PATH STATE MACHINE ==========
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Go back to near shot pose
                buildGoBackPath();
                follower.followPath(goBackPath, true);
                setPathState(1);
                break;

            case 1: // Wait to reach near shot pose
                if (!follower.isBusy()) {
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
//                intake.setPower(-1);
                manageSecondHopIntake();
                follower.followPath(bezierFirstPath, true);
                setPathState(4);
                break;

            case 4: // Wait for first bezier path
                if (!follower.isBusy()) {
                    manageSecondHopIntake();
                    follower.followPath(bezierSecondPath, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait for second bezier path
                manageSecondHopIntake();
                if (!follower.isBusy()) {
                    setActionState(1); // Start shooting
                    setPathState(6);
                }
                break;

            case 6: // Wait for shooting cycle 2
                if (actionState == 0) {
                    gateHitCount = 0; // Reset counter
                    setPathState(7); // Start gate cycles
                }
                break;

            // ===== GATE CYCLE LOOP (Repeats 3 times) =====
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
                    setPathState(9);
                }
                break;

            case 9: // Gate - pause to collect artifacts
                // DON'T manage intake while collecting at gate
                // We want to grab all 3 pieces first
                // Robot arrives with empty tray after shooting

                double waitTime2 = (gateHitCount == 0) ? GATE_WAIT_TIME_FIRST : GATE_WAIT_TIME_LATER;
                if (actionTimer.getElapsedTimeSeconds() > waitTime2) {
                    follower.followPath(gateSecondPath, true);
                    setPathState(10);
                }
                break;

            case 10: // Gate - return to shooting position
                manageSecondHopIntake();
                if (!follower.isBusy()) {
//                    intake.setPower(0);
                    setActionState(1); // Start shooting
                    setPathState(11);
                }
                break;

            case 11: // Wait for shooting to complete
                if (actionState == 0) {
                    gateHitCount++;

                    // Check if we need more gate cycles
                    if (gateHitCount < TOTAL_GATE_CYCLES) {
                        setPathState(7); // Loop back to gate cycle
                    } else {
                        setPathState(12); // Move to first line pickup
                    }
                }
                break;

            // ===== FIRST LINE PICKUP (replaces 3rd gate cycle) =====
            case 12: // Drive to first line pickup position
                intake.setPower(-1);
                buildFirstLinePickupPaths();
                follower.followPath(firstLinePickupPath, true);
                setPathState(13);
                break;

            case 13: // Second hop forward (13 inches)
                manageSecondHopIntake();
                if (!follower.isBusy()) {
                    follower.followPath(firstLineSecondHopPath, true);
                    setPathState(14);
                }
                break;

            case 14: // Second hop forward (8 inches)
                manageSecondHopIntake();
                if (!follower.isBusy()) {
//                    intake.setPower(0);
                    setPathState(15);
                }
                break;

            case 15: // Build return path and start driving back
                buildReturnToShootingPath();
                follower.followPath(goBackPath, true);
                setPathState(16);
                break;

            case 16: // Wait to return to shooting position
                if (!follower.isBusy()) {
                    setActionState(1); // Start final shooting
                    setPathState(17);
                }
                break;

            case 17: // Final shooting sequence
                if (actionState == 0) {
                    setPathState(-1); // All done!
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
                setActionState(2);
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
                    setActionState(0); // Done shooting - tray is now EMPTY
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
        if (t >= 0 && t < SHOOT_INTERVAL) {
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.allDown();
            LL.backUp();
        }
    }

    private void shootBLR() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL) {
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
            LL.leftUp();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.allDown();
            LL.rightUp();
        }
    }

    private void shootRBL() {
        double t = shootTimer.getElapsedTimeSeconds();
        if (t >= 0 && t < SHOOT_INTERVAL) {
            LL.rightUp();
        } else if (t >= SHOOT_INTERVAL && t < SHOOT_INTERVAL * 2) {
            LL.allDown();
            LL.backUp();
        } else if (t >= SHOOT_INTERVAL * 2 && t < SHOOT_INTERVAL * 3) {
            LL.allDown();
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
                .addPath(new Path(new BezierCurve(cur, midpoint1, firstpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), firstpickupPose.getHeading(), 0.8)
                .build();

        bezierSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(firstpickupPose, midpoint2, nearshotpose2)))
                .setLinearHeadingInterpolation(firstpickupPose.getHeading(), nearshotpose2.getHeading(), 0.8)
                .build();
    }

    private void buildGatePaths(double waitTime) {
        Pose cur = follower.getPose();
        gateFirstPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, infront_of_lever_new)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever_new.getHeading(), 0.5)
                .setTimeoutConstraint(1.2)
                .build();

        gateSecondPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(infront_of_lever_new, outfromgate, nearshotpose2)))
                .setLinearHeadingInterpolation(infront_of_lever_new.getHeading(), nearshotpose2.getHeading())
                .build();
    }

//    private void buildFirstLinePickupPaths() {
//        Pose cur = follower.getPose();
//
//        // First path: drive to first pickup position
//        firstLinePickupPath = follower.pathBuilder()
//                .addPath(new Path(new BezierLine(cur, firstPickupPose)))
//                .setLinearHeadingInterpolation(cur.getHeading(), firstPickupPose.getHeading(), 0.3)
//                .build();
//
//        // Second hop: move forward SECOND_HOP_IN + 7.5 inches
//        double heading = firstPickupPose.getHeading();
//        double dx = SECOND_HOP_IN * Math.cos(heading);
//        double dy = (SECOND_HOP_IN)+2 * Math.sin(heading);
//        Pose secondGoal = new Pose(
//                firstPickupPose.getX() + dx,
//                firstPickupPose.getY() + dy,
//                heading
//        );
//
//        firstLineSecondHopPath = follower.pathBuilder()
//                .addPath(new Path(new BezierLine(firstPickupPose, secondGoal)))
//                .setConstantHeadingInterpolation(heading)
//                .setTimeoutConstraint(0.2)
//                .build();
//    }
    private void buildFirstLinePickupPaths() {
        Pose cur = follower.getPose();

        // Drive straight to first pickup position
        firstLinePickupPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, firstPickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), firstPickupPose.getHeading())
                .build();
    }

    private void buildReturnToShootingPath() {
        Pose cur = follower.getPose();
        goBackPath = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1, nearshotpose2)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading())
                .build();
    }

    // ========== UTILITY METHODS ==========
    private void manageSecondHopIntake() {
        if (intake == null || LL == null || sensors == null) return;

        boolean rightFull = (sensors.getRight() != 0);
        boolean backFull = (sensors.getBack() != 0);
        boolean leftFull = (sensors.getLeft() != 0);

        int count = 0;
        if (rightFull) count++;
        if (backFull) count++;
        if (leftFull) count++;

        if (count >= 3) {
            intake.setPower(0.5); // Spit out
        } else {
            intake.setPower(-1); // Continue intake
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
    }
}