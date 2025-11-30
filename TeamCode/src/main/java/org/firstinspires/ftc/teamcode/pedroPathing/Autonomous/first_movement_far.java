package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import android.util.Size;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TagCoordinate;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto: Shoot 3 (delayed) → Move Path", group = "Pedro")
public class first_movement_far extends LinearOpMode {
    // THIS IS THE RED AUTO
    // THIS IS THE RED AUTO
    // THIS IS THE RED AUTO

    // ---------- Shooter subsystems -------------
    private Deposition depo;
    private launch_lift LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower;

    // ----- Voltage-comp power (tune these once around ~12.35V) -----
    private static final double FAR_BASE_POWER_12V   = 0.73;  // what worked for FAR at ~12.0V
    private static final double FAR_BASE_POWER2_12V  = 0.73;  // a touch hotter between shots (optional)
    private static final double CLOSE_BASE_POWER_12V = 0.55;  // what worked for CLOSE at ~12.0V

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private Pose startPose = new Pose(0.0, 0.0, Math.toRadians(-20.0));
    private Pose correctedStartPose = new Pose(0.0, 0.0, Math.toRadians(-20.0));

    // Your goal pose (exactly as in your movement program)
    private final Pose goalPose = new Pose(23, -24, Math.toRadians(-90.0));

    private final Pose mid_goal_1_Pose = new Pose(38.25, -15, Math.toRadians(-90.0));
    private final Pose goal_1_Pose     = new Pose(46.25, -23, Math.toRadians(-90.0));

    private final Pose mid_goal_2_Pose = new Pose(63.5, -15, Math.toRadians(-90.0));
    private final Pose goal_2_Pose     = new Pose(71.5, -24.5, Math.toRadians(-90.0));

    private final Pose near_shot_Pose  = new Pose(72, 0.5, Math.toRadians(-45.0));

    private static final double SECOND_HOP_IN = 13.0;
    private static final double SHOT_DELAY_S  = 0.75;  // delay between shots (you already use timing windows below)

    // ---------- Timing ----------
    private final ElapsedTime timer = new ElapsedTime();
    private double curTime  = 10000;
    public  double curTime2 = 10000;

    // --------- Voltage-comp helpers ---------
    private double getBatteryVoltage() {
        double v = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double vv = vs.getVoltage();
            if (vv > 0) v = Math.max(v, vv);
        }
        return (v > 0) ? v : 12.0;
    }

    /** Set both shooter motors with voltage compensation (base power is what you'd use at 12.0V). */
    private void setShooterPowerVoltageComp(double basePowerAt12V) {
        double v = getBatteryVoltage();          // e.g., 13.2V fresh, 12.0V nominal, 11.5V lower
        double compensated = basePowerAt12V * (12.35 / v);
        compensated = Math.max(0.0, Math.min(1.0, compensated));

        if (d1 != null) d1.setPower(compensated);
        if (d2 != null) d2.setPower(compensated);
        telemetry.addData("ShooterVComp", "V=%.2f base=%.3f out=%.3f", v, basePowerAt12V, compensated);
    }

    private void stopShooter() {
        if (d1 != null) d1.setPower(0.0);
        if (d2 != null) d2.setPower(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640,480)).build();
        TagCoordinate Tag20 = new TagCoordinate(5, 125, (float)29, -54);
        TagCoordinate Tag24 = new TagCoordinate(144-5, 125, (float)29, 54); //FIX THIS
        double fieldCentricRotation = 0;
        double fieldWiseY;
        double fieldWiseX;
        // Init subsystems
        depo    = new Deposition(hardwareMap);
        LL      = new launch_lift(hardwareMap);
        intake  = hardwareMap.get(DcMotor.class, "intake");
        d1      = hardwareMap.get(DcMotor.class, "depo");   // ensure names match RC config
        d2      = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: If one motor spins the wrong way, fix it here:
        // d2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Launcher safe start
        LL.down();
        LL.close();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay, voltage-comp) then run movement.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        if (!tagProcessor.getDetections().isEmpty()) {
            for(int i=0; i<tagProcessor.getDetections().size(); i++) {
                AprilTagDetection tag = tagProcessor.getDetections().get(i);
                try {
                    //normalizedRadians = (float)Math.toRadians(tag.ftcPose.bearing-tag.ftcPose.yaw);
                    //yOffset = Math.sin((double)normalizedRadians)*tag.ftcPose.range;
                    //xOffset = Math.cos((double)normalizedRadians)*tag.ftcPose.range;
                    telemetry.addData(i + " x", tag.ftcPose.x);
                    telemetry.addData(i + " y", tag.ftcPose.y);
                    telemetry.addData(i + " z", tag.ftcPose.z);
                    telemetry.addData(i + " roll", tag.ftcPose.roll);
                    telemetry.addData(i + " pitch", tag.ftcPose.pitch);
                    telemetry.addData(i + " yaw", tag.ftcPose.yaw);
                    telemetry.addData(i + " range", tag.ftcPose.range);
                    telemetry.addData(i + " bearing", tag.ftcPose.bearing);
                    if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                        telemetry.addData("OBELISK DETECTED", (tag.id == 21) ? "G P P" : (tag.id == 22) ? "P G P" : "P P G");
                    }
                    if (tag.id == 24) {
                        fieldCentricRotation = tag.ftcPose.yaw + Tag24.getTagYaw();
                        telemetry.addData("Field centric Rotation", fieldCentricRotation);
                    }

                    double currentX = tag.ftcPose.range * Math.sin(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw)); //THESE ARE FROM THE APRIL TAG AWAY
                    double currentY= tag.ftcPose.range * Math.cos(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw));
                    telemetry.addData("Tagwise X", currentX);
                    telemetry.addData("Tagwise Y", currentY);
                    // range * sin(bearing - yaw + tagRotation)
                    double TagFieldwiseY = tag.ftcPose.range * Math.cos(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw + Tag20.getTagYaw()));
                    double TagFieldwiseX = tag.ftcPose.range * Math.sin(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw + Tag20.getTagYaw()));
                    fieldWiseY = -(tag.ftcPose.range * Math.cos(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw + Tag20.getTagYaw()))) + Tag20.getTagY();
                    fieldWiseX = -(tag.ftcPose.range * Math.sin(Math.toRadians(-tag.ftcPose.bearing + tag.ftcPose.yaw + Tag20.getTagYaw()))) + Tag20.getTagX();
                    telemetry.addData("Relative Rotation", tag.ftcPose.bearing-tag.ftcPose.yaw);
                    telemetry.addData("Tag Fieldwise X", TagFieldwiseX);
                    telemetry.addData("Tag Fieldwise Y", TagFieldwiseY);
                    telemetry.addData("Fieldwise X", fieldWiseX);
                    telemetry.addData("Fieldwise Y", fieldWiseY);
                    telemetry.addData("ID number", tag.id);

                } catch (Exception e) {
                    telemetry.addData("exception: ", e);
                    telemetry.addData(i + " x", "NULL");
                    telemetry.addData(i + " y", "NULL");
                    telemetry.addData(i + " z", "NULL");
                    telemetry.addData(i + " roll", "NULL");
                    telemetry.addData(i + " pitch", "NULL");
                    telemetry.addData(i + " yaw", "NULL");

                }
            }
        }
        telemetry.update();
        TimeUnit.SECONDS.sleep(2);
        startPose = new Pose(startPose.getX(), startPose.getY(), -Math.toRadians(fieldCentricRotation));
        follower.setPose(startPose);
        telemetry.addData("StartPose", startPose);
        telemetry.update();
        TimeUnit.SECONDS.sleep(5);
        correct_heading();
        three_far_shots();
        first_line_pickup();
        reset();
        go_home();
        three_far_shots();
        second_line_pickup();
        reset();
        go_close();
        three_close_shots();
        third_line_pickup();
        reset();
        go_close();
        three_close_shots();

        telemetry.addLine("✅ Done: fired 3 shots (with delay) + completed both paths.");
        telemetry.update();
        sleep(500);
    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);
        LL.down();
        LL.close();
        if (intake != null) intake.setPower(0);
    }

    private void correct_heading() {
        Pose cur = follower.getPose();
        PathChain correction = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, correctedStartPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), correctedStartPose.getHeading())
                .build();
        follower.followPath(correction, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }
    private void go_home() {
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, correctedStartPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), correctedStartPose.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    // ===== Far / Close shot sequence starters =====
    private void startFarShot() {
        curTime2 = timer.seconds();
        LL.far();
        // Spin up to FAR power (voltage-compensated)
        setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
    }

    private void startCloseShot() {
        curTime2 = timer.seconds();
        LL.close();
        // Spin up to CLOSE power (voltage-compensated)
        setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V);
    }

    private void three_far_shots() {
        startFarShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            farshoot3x();   // uses voltage-comp calls below
            follower.update();
        }
    }

    private void three_close_shots() {
        startCloseShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            closeshoot3x(); // uses voltage-comp calls below
            follower.update();
        }
    }

    private void first_line_pickup() {
        if (intake != null) intake.setPower(-1);
        // path 1
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, goalPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goalPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        // path 2 (13" forward)
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = SECOND_HOP_IN * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        follower.setMaxPower(0.5);
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        follower.setMaxPower(1.0);
    }

    private void second_line_pickup() {
        if (intake != null) intake.setPower(-1);
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(startPose, mid_goal_1_Pose, goal_1_Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goal_1_Pose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = SECOND_HOP_IN * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }

    private void third_line_pickup() {
        if (intake != null) intake.setPower(-1);
        Pose cur = follower.getPose();
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, mid_goal_2_Pose, goal_2_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), goal_2_Pose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur1 = follower.getPose();
        double heading = cur1.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = SECOND_HOP_IN * Math.sin(heading);
        Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);

        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur1, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }

    private void go_close() {
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }

    private boolean isFarShotCycleDone() { return (curTime2 == 10000); }

    // ---------- Original timing windows, but with voltage-comp shooter power ----------

    public void closeshoot3x() {
        // spin-up already set in startCloseShot()
        if (timer.seconds() >= curTime2 && timer.seconds() < curTime2 + 0.1) {
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V);
            LL.close();
        }
        if (timer.seconds() >= curTime2 + 1.0 && timer.seconds() < curTime2 + 1.1) LL.up();
        if (timer.seconds() >= curTime2 + 1.3 && timer.seconds() < curTime2 + 1.4) LL.down();

        // intake 1
        if (timer.seconds() >= curTime2 + 1.5 && timer.seconds() < curTime2 + 1.6) {
            if (intake != null) intake.setPower(-1);
        }
        // shoot 2
        if (timer.seconds() >= curTime2 + 2.0 && timer.seconds() < curTime2 + 2.1) {
            LL.up();
            if (intake != null) intake.setPower(0);
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V); // keep topped up
        }
        if (timer.seconds() >= curTime2 + 2.3 && timer.seconds() < curTime2 + 2.4) LL.down();

        // intake 2
        if (timer.seconds() >= curTime2 + 2.5 && timer.seconds() < curTime2 + 2.6) {
            if (intake != null) intake.setPower(-1);
        }
        // shoot 3
        if (timer.seconds() >= curTime2 + 3.0 && timer.seconds() < curTime2 + 3.1) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer.seconds() >= curTime2 + 3.3 && timer.seconds() < curTime2 + 3.4) {
            LL.down();
            stopShooter();
            curTime2 = 10000;
        }
    }

    public void farshoot3x() {
        // shot 1
        if (timer.seconds() >= curTime2 && timer.seconds() < curTime2 + 0.1) {
            setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
            LL.far();
        }
        if (timer.seconds() >= curTime2 + 1.0 && timer.seconds() < curTime2 + 1.1) LL.up();
        if (timer.seconds() >= curTime2 + 1.3 && timer.seconds() < curTime2 + 1.4) LL.down();

        // feed 1 and slightly bump power if you used to
        if (timer.seconds() >= curTime2 + 1.5 && timer.seconds() < curTime2 + 1.6) {
            if (intake != null) intake.setPower(-1);
            setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
        }

        // shot 2
        if (timer.seconds() >= curTime2 + 2.0 && timer.seconds() < curTime2 + 2.1) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer.seconds() >= curTime2 + 2.3 && timer.seconds() < curTime2 + 2.4) LL.down();

        // feed 2 + return to main FAR power
        if (timer.seconds() >= curTime2 + 2.5 && timer.seconds() < curTime2 + 2.6) {
            if (intake != null) intake.setPower(-1);
            setShooterPowerVoltageComp(FAR_BASE_POWER2_12V);
        }

        // shot 3
        if (timer.seconds() >= curTime2 + 3.0 && timer.seconds() < curTime2 + 3.1) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer.seconds() >= curTime2 + 3.3 && timer.seconds() < curTime2 + 3.4) {
            LL.down();
            stopShooter();
            curTime2 = 10000;
        }
    }
}
