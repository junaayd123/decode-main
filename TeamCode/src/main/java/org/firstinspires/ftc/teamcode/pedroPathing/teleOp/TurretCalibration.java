package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Turret Calibration", group = "A_TeleOp")
public class TurretCalibration extends OpMode {

    private Follower follower;
    private TurretLimelight turret;
    private VisionSubsystem vision;
    private lifters lift;

    private boolean tagInitializing = false;
    private double totalHedOffset = 0;
    private Timer turretTimer = new Timer();
    
    private double manualTurretTicks = 0;
    private final Pose redGoal = new Pose(72, 144, 0);
    private final Pose startPose = new Pose(53, 70, 0);

    private static class TurretDataPoint {
        Pose robotPose;
        double actualTicks;
        double theoreticalDeg;
        double errorDeg;

        TurretDataPoint(Pose p, double ticks, double theoretical) {
            this.robotPose = p;
            this.actualTicks = ticks;
            this.theoreticalDeg = theoretical;
            // 670 ticks = 180 degrees
            double actualDeg = ticks / (670.0 / 180.0);
            this.errorDeg = actualDeg - theoreticalDeg;
        }
    }

    private List<TurretDataPoint> dataPoints = new ArrayList<>();
    private String regressionResult = "Need 2+ points";

    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        turret = new TurretLimelight(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        lift = new lifters(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lift.allDown();
        lift.set_camera_tag_pos();
    }

    @Override
    public void loop() {
        vision.update();
        turret.updateEncoderPos();
        follower.update();

        Pose cur = follower.getPose();

        handleLocalization();
        handleManualTurret();
        handleDrive();
        handleDataCollection(cur);

        doTelemetry(cur);
    }

    private void handleLocalization() {
        if (gamepad1.triangleWasPressed()) {
            if (tagInitializing) {
                tagInitializing = false;
                vision.setAprilTagEnabled(false);
            } else {
                turretTimer.startTimer();
                turret.toTargetInDegrees2(0);
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
                follower.setPose(localized);
                totalHedOffset = follower.getTotalHeading() - localized.getHeading();
                tagInitializing = false;
                vision.setAprilTagEnabled(false);
            }
        }
    }

    private void handleManualTurret() {
        // Adjust turret ticks manually with G2 Dpad
        if (gamepad2.dpad_left)  manualTurretTicks -= 2;
        if (gamepad2.dpad_right) manualTurretTicks += 2;
        
        // Clamp to physical limits
        manualTurretTicks = Math.max(-670, Math.min(670, manualTurretTicks));
        
        turret.toTargetInTicks(); // This usually uses turret.target
        // But let's use the safer API:
        turret.setTicksTarget((int)manualTurretTicks);
        turret.toTargetInTicks();
    }

    private void handleDrive() {
        double speed = gamepad1.cross ? 0.3 : 1.0;
        follower.setTeleOpDrive(-gamepad1.left_stick_y * speed,
                (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                -gamepad1.right_stick_x * speed, true);
    }

    private void handleDataCollection(Pose cur) {
        if (gamepad2.squareWasPressed()) {
            double headingToGoal = calculateHeadingTo(cur, redGoal);
            double robHeading = normalizeAngle(follower.getTotalHeading() - totalHedOffset);
            double theoreticalDeg = Math.toDegrees(robHeading - headingToGoal);
            
            dataPoints.add(new TurretDataPoint(cur, manualTurretTicks, theoreticalDeg));
            calculateRegression();
        }
    }

    private void calculateRegression() {
        if (dataPoints.size() < 2) return;

        // We want to find a linear regression for the Error Offset vs Distance
        // or just a constant offset.
        double n = dataPoints.size();
        double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;

        for (TurretDataPoint p : dataPoints) {
            double dist = p.robotPose.distanceFrom(redGoal);
            sumX += dist;
            sumY += p.errorDeg;
            sumXX += dist * dist;
            sumXY += dist * p.errorDeg;
        }

        double denom = (n * sumXX - sumX * sumX);
        if (Math.abs(denom) < 1e-9) {
            // If all at same distance, just average the error
            regressionResult = String.format(Locale.US, "Avg Offset: %.2f deg", sumY / n);
            return;
        }

        double m = (n * sumXY - sumX * sumY) / denom;
        double b = (sumY - m * sumX) / n;
        regressionResult = String.format(Locale.US, "Offset: y = %.4fx + %.2f", m, b);
    }

    private double calculateHeadingTo(Pose cur, Pose target) {
        double raw = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());
        return normalizeAngle(raw + Math.PI);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void doTelemetry(Pose cur) {
        telemetry.addLine("--- TURRET CALIBRATION ---");
        telemetry.addData("G1 Triangle", "Localize (Hold)");
        telemetry.addData("G2 Dpad L/R", "Manual Ticks: %.1f", manualTurretTicks);
        telemetry.addData("G2 Square", "SAVE DATA POINT");
        
        telemetry.addLine("\n--- REGRESSION ---");
        telemetry.addData("Points", dataPoints.size());
        telemetry.addLine(regressionResult);
        
        telemetry.addLine("\n--- LIVE DATA ---");
        double headingToGoal = calculateHeadingTo(cur, redGoal);
        double robHeading = normalizeAngle(follower.getTotalHeading() - totalHedOffset);
        double theoreticalDeg = Math.toDegrees(normalizeAngle(robHeading - headingToGoal));
        double actualDeg = manualTurretTicks / (670.0 / 180.0);
        
        telemetry.addData("Robot Pos", "X: %.1f, Y: %.1f", cur.getX(), cur.getY());
        telemetry.addData("Dist to Goal", "%.1f", cur.distanceFrom(redGoal));
        telemetry.addData("Theo Deg", "%.2f", theoreticalDeg);
        telemetry.addData("Actual Deg", "%.2f", actualDeg);
        telemetry.addData("Current Error", "%.2f", actualDeg - theoreticalDeg);
        
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.close();
    }
}
