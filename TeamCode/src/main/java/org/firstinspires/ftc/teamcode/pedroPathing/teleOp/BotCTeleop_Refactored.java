package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
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

@TeleOp(name = "Bot C Teleop Refactored", group = "A_TeleOp")
public class BotCTeleop_Refactored extends OpMode {

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
    private enum Mode { nothing, findTag, faceGoal, faceRamp }
    private Mode mode = Mode.nothing;
    private Mode modeBeforeRampScan = Mode.nothing;

    private boolean aligning = false;
    private double distanceToGoal;
    private double desiredHeading = 0;
    private String motif = "gpp";
    private int ballOnRamp;
    private int greenInSlot;
    private boolean shootingTest = false;
    private double ourVelo = 1300;
    private double totalHedOffset;
    private double speed = 1.0;
    private boolean tagInitializing = false;
    
    private final Pose redGoal = new Pose(62, 137, 0);
    private final Pose redGoalFixed = new Pose(72, 144, 0);
    private final Pose redGoalfar = new Pose(62, 140, 0);
    private final Pose rampPose = new Pose(72, 80, 0);
    private final Pose startPose = new Pose(53, 70, 0);

    private Timer turretTimer = new Timer();
    private Timer rampScanDelayTimer = new Timer();
    private Servo led, led2;

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

        handleRampScan();
        handleTurretMode(cur);
        handleTagLocalization();
        handleIntake();
        handleShooting();
        handleDrive();

        updateLEDs();
        doTelemetry(cur);
    }

    private void handleRampScan() {
        if (gamepad1.bWasPressed() && !vision.isRampScanning()) {
            modeBeforeRampScan = mode;
            mode = Mode.faceRamp;
            vision.startRampScan();
            rampScanDelayTimer.startTimer();
        }

        if (vision.isRampScanning()) {
            if (rampScanDelayTimer.checkAtSeconds(1.0)) {
                lift.allDown();
            }
        } else if (rampScanDelayTimer.timerIsOn()) {
            if (rampScanDelayTimer.checkAtSeconds(1.0)) {
                mode = modeBeforeRampScan;
                ballOnRamp = vision.getRampBallVerdict() % 3;
                greenInSlot = getGreenPos();
            }
            if (rampScanDelayTimer.checkAtSeconds(1.3)) {
                rampScanDelayTimer.stopTimer();
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
    }

    private void handleTurretMode(Pose cur) {
        Pose target = (distanceToGoal > 125) ? redGoalfar : redGoal;
        double headingToTarget = calculateHeadingTo(cur, target);
        double robHeading = normalizeAngle(follower.getTotalHeading() - totalHedOffset);

        switch (mode) {
            case faceGoal:
                turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingToTarget));
                break;
            case findTag:
                turret.toTargetInDegrees();
                break;
            case faceRamp:
                double headingToRamp = calculateHeadingTo(cur, rampPose);
                double rampDeg = Math.toDegrees(robHeading - headingToRamp);
                while (rampDeg > 180) rampDeg -= 360;
                while (rampDeg < -180) rampDeg += 360;
                turret.toTargetInDegrees2(rampDeg);
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
            Pose localized = vision.getLocalizedPose(turret.currentPos); // Assuming turretDeg = turret.currentPos
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
//        if (gamepad2.dpadDownWasPressed()) shootingTest = !shootingTest;
        if (gamepad2.psWasPressed()) {
            if (motif.equals("gpp")) motif = "pgp";
            else if (motif.equals("pgp")) motif = "ppg";
            else motif = "gpp";
        }

        if (gamepad2.dpadLeftWasPressed()) shooter.stop();

        if (gamepad2.crossWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp = 0;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
        if (gamepad2.squareWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp = 0;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
        if (gamepad2.triangleWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp = 1;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
        if (gamepad2.circleWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp = 2;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }

        if (!shootingTest && !shooter.isShooting()) {
            lift.set_angle_custom(reg.distanceToAngle(distanceToGoal));
        }
        
        if (gamepad1.dpadUpWasPressed()) ourVelo += 20;
        else if (gamepad1.dpadDownWasPressed()) ourVelo -= 20;
    }

    private void handleDrive() {
        speed = gamepad1.cross ? 0.3 : 1.0;
        if (!follower.isBusy() && !aligning) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y * speed,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                    -gamepad1.right_stick_x * speed, true);
        }
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
        telemetry.addData("motif", motif);
        telemetry.addData("shooter sequence", shooter.getShooterSequence());
        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("actual depo velo", depo.getVelocity());
        telemetry.addData("target velocity", shootingTest ? ourVelo : depo.targetVelocity);
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("Ramp ball verdict", vision.getRampBallVerdict());
        telemetry.update();
    }
}
