package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.IntakeManager;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ShooterManager;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.regressions;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Regression Calibration", group = "A_TeleOp")
public class RegressionCalibration extends OpMode {

    // Subsystems
    private Follower follower;
    private TurretLimelight turret;
    private lifters lift;
    private Deposition_C depo;
    private IntakeManager intake;
    private VisionSubsystem vision;
    private ShooterManager shooter;
    private regressions reg;

    // State Variables
    private enum Mode { nothing, findTag, faceGoal }
    private Mode mode = Mode.nothing;

    private double distanceToGoal;
    private String motif = "gpp";
    private double ourVelo = 1600;
    private double manualAngle = 0.1;
    private double totalHedOffset;
    private double speed = 1.0;
    private boolean tagInitializing = false;

    // Goals
    private Pose redGoal = new Pose(62, 137, 0); // Modifiable via G2 Dpad for Turret Alignment
    private final Pose redGoalFixed = new Pose(72, 144, 0); // Fixed for Distance Calculation
    private final Pose redGoalfar = new Pose(62, 140, 0);
    private final Pose startPose = new Pose(53, 70, 0);

    private Timer turretTimer = new Timer();
    private Servo led, led2;

    // Data Logging
    private static class DataPoint {
        double dist, velo, angle;
        DataPoint(double d, double v, double a) { dist = d; velo = v; angle = a; }
    }
    private List<DataPoint> dataPoints = new ArrayList<>();
    private String veloRegString = "Need 2+ points";
    private String angleRegString = "Need 2+ points";

    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new TurretLimelight(hardwareMap);
        lift = new lifters(hardwareMap);
        depo = new Deposition_C(hardwareMap);
        reg = new regressions();
        intake = new IntakeManager(hardwareMap, lift.sensors);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterManager(depo, lift, reg);

        led = hardwareMap.get(Servo.class, "led");
        led2 = hardwareMap.get(Servo.class, "led2");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lift.allDown();
        lift.set_angle_min();
        lift.set_camera_tag_pos();
    }

    @Override
    public void loop() {
        vision.update();
        shooter.update();
        intake.update();
        turret.updateEncoderPos();
        follower.update();

        Pose cur = follower.getPose();
        distanceToGoal = cur.distanceFrom(redGoalFixed);

        handleTurretMode(cur);
        handleTagLocalization();
        handleIntake();
        handleShooting();
        handleCalibration();
        handleDrive();

        updateLEDs();
        doTelemetry(cur);
    }

    @Override
    public void stop() {
        if (vision != null) {
            vision.close();
        }
    }

    private void handleTurretMode(Pose cur) {
        Pose target =redGoal;
        double headingToTarget = calculateHeadingTo(cur, target);
        double robHeading = normalizeAngle(follower.getTotalHeading() - totalHedOffset);

        switch (mode) {
            case faceGoal:
                if (distanceToGoal > 125) {
                    turret.toTargetInDegrees2(Math.toDegrees(robHeading) + reg.getRedTurretFar(cur.getX(),cur.getY(),cur.getHeading()));
                } else {
                    turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingToTarget));
                }
                break;
            case findTag:
                turret.toTargetInDegrees();
                break;
        }
    }

    private void handleTagLocalization() {
        if (gamepad1.triangleWasPressed()) {
            if (tagInitializing) {
                tagInitializing = false;
                mode = Mode.nothing;
                vision.setAprilTagEnabled(false);
            } else {
                turretTimer.startTimer();
                mode = Mode.findTag;
                turret.setDegreesTarget(0);
                vision.setAprilTagEnabled(true);
            }
        }

        if (turretTimer.checkAtSeconds(1)) {
            tagInitializing = true;
            turretTimer.stopTimer();
        }

        if (tagInitializing) {
            Pose localized = vision.getLocalizedPose(turret.currentPos);
            if (localized != null) {
                mode = Mode.faceGoal;
                follower.setPose(localized);
                totalHedOffset = follower.getTotalHeading() - localized.getHeading();
                tagInitializing = false;
                vision.setAprilTagEnabled(false);
            }
        }
    }

    private void handleIntake() {
        if (gamepad2.rightBumperWasPressed()) {
            lift.allDown();
            if (intake.isCollecting()) intake.stop();
            else intake.startCollecting();
        }

        if (shooter.isShooting()) intake.stop();

        if (gamepad2.left_bumper) intake.manualReverse();
        else if (!gamepad2.left_bumper && !intake.isCollecting() && !intake.isReversing()) intake.manualStop();
    }

    private void handleShooting() {
        // Manual Velo Control (Gamepad 1 Dpad Up/Down)
        if (gamepad1.dpadUpWasPressed()) ourVelo += 20;
        if (gamepad1.dpadDownWasPressed()) ourVelo -= 20;

        // Manual Angle Control (Gamepad 1 Dpad Left/Right)
        if (gamepad1.dpadRightWasPressed()) manualAngle += 0.01;
        if (gamepad1.dpadLeftWasPressed()) manualAngle -= 0.01;

        // Stop Shot (Gamepad 2 Circle)
        if (gamepad2.circleWasPressed()) shooter.stop();

        // Shoot (Gamepad 2 Cross)
        if (gamepad2.crossWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, true, ourVelo);
                lift.set_angle_custom(manualAngle); // Explicitly set manual angle
                shooter.startShooting(motif, 0, getGreenPos());
            }
        }

        // Apply manual angle visualization when not in sequence
        if (!shooter.isShooting()) {
            lift.set_angle_custom(manualAngle);
        }
    }

    private void handleCalibration() {
        // Adjust redGoal for Turret Alignment (Gamepad 2 Dpad)
        if (gamepad2.dpadUpWasPressed()) redGoal = new Pose(redGoal.getX(), redGoal.getY() + 1, 0);
        if (gamepad2.dpadDownWasPressed()) redGoal = new Pose(redGoal.getX(), redGoal.getY() - 1, 0);
        if (gamepad2.dpadLeftWasPressed()) redGoal = new Pose(redGoal.getX() - 1, redGoal.getY(), 0);
        if (gamepad2.dpadRightWasPressed()) redGoal = new Pose(redGoal.getX() + 1, redGoal.getY(), 0);

        // Save Data Point (Gamepad 2 Square)
        if (gamepad2.squareWasPressed()) {
            dataPoints.add(new DataPoint(distanceToGoal, ourVelo, manualAngle));
            calculateRegressions();
        }
    }

    private void calculateRegressions() {
        if (dataPoints.size() < 2) return;

        double n = dataPoints.size();
        double sumX = 0, sumYV = 0, sumYA = 0, sumXX = 0, sumXYV = 0, sumXYA = 0;

        for (DataPoint p : dataPoints) {
            sumX += p.dist;
            sumYV += p.velo;
            sumYA += p.angle;
            sumXX += p.dist * p.dist;
            sumXYV += p.dist * p.velo;
            sumXYA += p.dist * p.angle;
        }

        double denom = (n * sumXX - sumX * sumX);
        if (Math.abs(denom) < 1e-9) return;

        // Velocity Regression
        double mV = (n * sumXYV - sumX * sumYV) / denom;
        double bV = (sumYV - mV * sumX) / n;
        veloRegString = String.format(Locale.US, "Velo: y = %.4fx + %.2f", mV, bV);

        // Angle Regression
        double mA = (n * sumXYA - sumX * sumYA) / denom;
        double bA = (sumYA - mA * sumX) / n;
        angleRegString = String.format(Locale.US, "Angle: y = %.6fx + %.4f", mA, bA);
    }

    private void handleDrive() {
        speed = gamepad1.cross ? 0.3 : 1.0;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * speed,
                (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                -gamepad1.right_stick_x * speed, true);
    }

    private void updateLEDs() {
        if (intake.isCollecting()) {
            led.setPosition(0.28);
            led2.setPosition(0.28);
        } else if (intake.getBallCount() >= 3) {
            led.setPosition(0.5);
            led2.setPosition(0.5);
        } else if (mode == Mode.findTag) {
            led.setPosition(0.34);
            led2.setPosition(0.34);
        } else if (mode == Mode.faceGoal && !tagInitializing) {
            led.setPosition(0.6);
            led2.setPosition(0.6);
        } else {
            led.setPosition(0);
            led2.setPosition(0);
        }
    }

    private double calculateHeadingTo(Pose cur, Pose target) {
        double raw = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());
        double flipped = raw + Math.PI;
        flipped = ((flipped + Math.PI) % (2 * Math.PI)) - Math.PI;
        return flipped + Math.PI;
    }

    private double normalizeAngle(double angle) {
        while (angle >= Math.toRadians(210)) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private int getGreenPos() {
        if (lift.sensors.getLeft() == 1) return 0;
        return (lift.sensors.getRight() == 1) ? 2 : 1;
    }

    private void doTelemetry(Pose cur) {
        telemetry.addLine("--- CALIBRATION CONTROLS ---");
        telemetry.addData("G1 Dpad Up/Down", "Velo: %.1f", ourVelo);
        telemetry.addData("G1 Dpad Left/Right", "Angle: %.4f", manualAngle);
        telemetry.addData("G2 Dpad", "Adjust Turret Target Pose");
        telemetry.addData("G2 Square", "LOG DATA POINT");
        telemetry.addData("G2 Circle", "STOP SHOOTER");
        telemetry.addData("G2 Cross", "SHOOT MANUAL");

        telemetry.addLine("\n--- REGRESSIONS ---");
        telemetry.addData("Points Saved", dataPoints.size());
        telemetry.addLine(veloRegString);
        telemetry.addLine(angleRegString);

        telemetry.addLine("\n--- ROBOT STATE ---");
        telemetry.addData("Distance to Goal (Fixed)", String.format(Locale.US, "%.2f", distanceToGoal));
        telemetry.addData("Turret Target Pose", "X: %.2f, Y: %.2f", redGoal.getX(), redGoal.getY());
        telemetry.addData("Actual Depo Velo", depo.getVelocity());
        telemetry.addData("Robot Pos", "X: %.2f, Y: %.2f", cur.getX(), cur.getY());
        telemetry.update();
    }
}
