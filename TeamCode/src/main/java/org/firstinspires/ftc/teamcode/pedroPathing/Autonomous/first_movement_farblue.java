package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;


@Autonomous(name = "blueFarAuto", group = "Pedro")
public class first_movement_farblue extends LinearOpMode {

    // ---------- Shooter subsystems -------------
    private Deposition depo;
    private launch_lift LL;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;
    // ---------- Pedro ----------
    private Follower follower;
    private static final double FAR_BASE_POWER_12V   = 0.715;  // what worked for FAR at ~12.0V
    private static final double FAR_BASE_POWER2_12V  = 0.715;  // a touch hotter between shots (optional)
    private static final double CLOSE_BASE_POWER_12V = 0.59;  // what worked for CLOSE at ~12.0V
    private static final double CLOSE_BASE_POWER2_12V = 0.59;  // what worked for CLOSE at ~12.0V

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)

    private final Pose start_align_Pose = new Pose(-4.0, -3.0, Math.toRadians(0.0));
    private final Pose startPose = new Pose(
            0.0,                     // x inches
            0.0,                     // y inches
            Math.toRadians(25.0)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose goalPose = new Pose(
            23,                    // x inches (forward)
            22,                   // y inches (left) og: 24
            Math.toRadians(90.0)    // heading (rad) at finish
    );

    private final Pose mid_goal_1_Pose = new Pose(
            38.25,                    // x inches (forward)
            15,                   // y inches (left)
            Math.toRadians(90.0)    // heading (rad) at finish
    );
    private final Pose goal_1_Pose = new Pose(
            46.25,                    // x inches (forward)
            21,                   // y inches (left) og: 23
            Math.toRadians(90.0)    // heading (rad) at finish
    );

    private final Pose mid_goal_2_Pose = new Pose(
            63.5,                    // x inches (forward)
            15,                   // y inches (left)
            Math.toRadians(90.0)    // heading (rad) at finish
    );
    private final Pose goal_2_Pose = new Pose(
            71,                    // x inches (forward)
            21.5,            // y inches (left) og: 24.5
            Math.toRadians(90.0)    // heading (rad) at finish
    );


    private final Pose near_shot_Pose = new Pose(
            71.5,                    // x inches (forward)
            -0.5,                   // y inches (left)
            Math.toRadians(45.0)    // heading (rad) at finish
    );
    private final Pose infront_of_lever   = new Pose(61.5, 37.5, Math.toRadians(0));

    private static final double SECOND_HOP_IN = 13.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)

    // ---------- Timing for far shots ----------
    private final ElapsedTime timer = new ElapsedTime();
    private double curTime = 10000;
    public double curTime2 = 10000;

    private double getBatteryVoltage() {
        double v = 0.0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor vs : hardwareMap.voltageSensor) {
            double vv = vs.getVoltage();
            if (vv > 0) v = Math.max(v, vv);
        }
        return (v > 0) ? v : 12.35;
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
        follower.setStartingPose(start_align_Pose);

        // Launcher safe start
        LL.down();
        LL.far();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay, voltage-comp) then run movement.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        first_align_movement();
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
        go_infront();
        // go_close();
        // three_close_shots();

        telemetry.addLine("✅ Done: fired 3 shots (with delay) + completed both paths.");
        telemetry.update();
        sleep(500);
    }

    private void first_align_movement() {
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(start_align_Pose, startPose)))
                .setLinearHeadingInterpolation(start_align_Pose.getHeading(), startPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }
    private void go_infront(){
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(),infront_of_lever.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);
        LL.down();
        LL.close();
        if (intake != null) intake.setPower(0);
    }

    private void go_home() {
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, startPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), startPose.getHeading())
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
        intake.setPower(0);
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
        intake.setPower(0);
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
        intake.setPower(0);
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

    public void closeshoot3x(){
        if (timer.seconds() >= curTime2 && timer.seconds() < curTime2 + 0.1) {
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V); // set power right before the shot
            LL.close();                                       // ensure angle/hood is in close position
        }
        if (timer.seconds() >= curTime2 + 1.0 && timer.seconds() < curTime2 + 1.1) LL.up();
        if (timer.seconds() >= curTime2 + 1.3 && timer.seconds() < curTime2 + 1.4) LL.down();

        // Feed 1 (optional: keep power at CLOSE_BASE or bump slightly)
        if (timer.seconds() >= curTime2 + 1.5 && timer.seconds() < curTime2 + 1.6) {
            if (intake != null) intake.setPower(-1);
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V); // hold/restore power during feed
        }

        // ---- Shot 2 ----
        if (timer.seconds() >= curTime2 + 2.0 && timer.seconds() < curTime2 + 2.1) {
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V); // set again right before shot 2
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer.seconds() >= curTime2 + 2.3 && timer.seconds() < curTime2 + 2.4) LL.down();

        // Feed 2 (optionally bump slightly with CLOSE_BASE_POWER2_12V)
        if (timer.seconds() >= curTime2 + 2.5 && timer.seconds() < curTime2 + 2.6) {
            if (intake != null) intake.setPower(-1);
            setShooterPowerVoltageComp(CLOSE_BASE_POWER2_12V);
        }

        // ---- Shot 3 ----
        if (timer.seconds() >= curTime2 + 3.0 && timer.seconds() < curTime2 + 3.1) {
            setShooterPowerVoltageComp(CLOSE_BASE_POWER_12V+0.01); // set again right before shot 3
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer.seconds() >= curTime2 + 3.3 && timer.seconds() < curTime2 + 3.4) {
            LL.down();
            depo.setPowerBoth(0.0);
            stopShooter();
            curTime2 = 10000; // sequence done
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


/* // ---------------------- Shooter subsystems --------------------
    private Deposition depo;
    private launch_lift LL;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;
    // ---------- Pedro ----------
    private Follower follower;

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose startPose = new Pose(
            0.0,                     // x inches
            0.0,                     // y inches
            Math.toRadians(20.0)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose goalPose = new Pose(
            23,                    // x inches (forward)
            24,                   // y inches (right)
            Math.toRadians(90.0)    // heading (rad) at finish
    );

    private final Pose mid_goal_1_Pose = new Pose(
            38.25,                    // x inches (forward)
            15,                   // y inches (right)
            Math.toRadians(90.0)    // heading (rad) at finish
    );
    private final Pose goal_1_Pose = new Pose(
            46.25,                    // x inches (forward)
            23,                   // y inches (right)
            Math.toRadians(90.0)    // heading (rad) at finish
    );

    private final Pose mid_goal_2_Pose = new Pose(
            63.5,                    // x inches (forward)
            15,                   // y inches (right)
            Math.toRadians(90.0)    // heading (rad) at finish
    );
    private final Pose goal_2_Pose = new Pose(
            71.5,                    // x inches (forward)
            24,            // y inches (right)
            Math.toRadians(90.0)    // heading (rad) at finish
    );


    private final Pose near_shot_Pose = new Pose(
            80.75,                    // x inches (forward)
            -0.5,                   // y inches (right)
            Math.toRadians(45.0)    // heading (rad) at finish
    );

    private static final double SECOND_HOP_IN = 13.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)*/
