package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

@Autonomous(name = "newBot_farRed", group = "Pedro")
public class newBot_farRed extends LinearOpMode {

    // ---------- Shooter subsystems -------------
    // Commented out: depo usage will be removed
    private Deposition depo;
    private lift_three LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower; //hi

    // ----- Voltage-comp power (tune these once around ~12.35V) -----
    private static final double FAR_BASE_POWER_12V   = 0.67;   // what worked for FAR at ~12.0V
    private static final double FAR_BASE_POWER2_12V  = 0.675;  // a touch hotter between shots (optional)
    private static final double CLOSE_BASE_POWER_12V = 0.55;   // what worked for CLOSE at ~12.0V
    private static final double CLOSE_BASE_POWER2_12V = 0.55;  // what worked for CLOSE at ~12.0V

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose start_align_Pose = new Pose(-4.0, 2, Math.toRadians(-180));
    private final Pose startPose = new Pose(0.0, 0.0, Math.toRadians(-201.5));

    // Your goal pose (exactly as in your movement program)
    private final Pose firstpickupPose = new Pose(24, -20, Math.toRadians(-90));

    private final Pose midPoint1 = new Pose(37, -14, Math.toRadians(-90));
    private final Pose secondpickupPose = new Pose(47, -17, Math.toRadians(-90));

    private final Pose midPoint2 = new Pose(44, -4, Math.toRadians(-90));
    private final Pose thirdpickupPose = new Pose(73, -20, Math.toRadians(-90));
    private final Pose midPoint3 = new Pose(76, -4, Math.toRadians(-90));
    private final Pose near_shot_Pose  = new Pose(97.5, -17, Math.toRadians(-240.0));
    private final Pose infront_of_lever   = new Pose(61.5, -37.5, Math.toRadians(0));

    private static final double SECOND_HOP_IN = 13.5;
    private static final double SHOT_DELAY_S  = 0.75;  // delay between shots (you already use timing windows below)

    // ---------- Upgraded shooter timing ----------
    private Timer timer1;
    private int sequence = 0;

    // --------- Voltage-comp helpers (kept) ---------
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
        depo    = new Deposition(hardwareMap);  // COMMENTED OUT (depo)
        LL      = new lift_three(hardwareMap);
        timer1  = new Timer();

        intake  = hardwareMap.get(DcMotor.class, "intake");  // COMMENTED OUT (intake)
        d1      = hardwareMap.get(DcMotor.class, "depo");   // ensure names match RC config
        d2      = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pedro follower (this is fine for Pedro 2.0)
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(start_align_Pose);

        // Launcher safe start
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with depo PID + timer3) then run movement.");
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
        go_close_2();
        three_close_shots();
        reset();
        go_infront();

        telemetry.addLine("✅ Done: fired shots + completed paths.");
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

    private void go_infront() {
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur,infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);              // COMMENTED OUT (depo)

        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)
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

    // ===== Far / Close shot sequence starters (upgraded) =====
    private void startFarShot() {
        sequence = 3;
        depo.setTargetVelocity(depo.farVelo_New);  // COMMENTED OUT (depo)
//        LL.far();
        // optionally: setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
    }

    private void startCloseShot() {
        sequence = 4;
        depo.setTargetVelocity(depo.closeVelo_New);  // COMMENTED OUT (depo)
//        LL.close();

    }

    private void three_far_shots() {
        LL.set_angle_far_auto();
        startFarShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();  // COMMENTED OUT (depo)
            if (depo.reachedTarget()) {  // COMMENTED OUT (depo)
                if (sequence == 3 || sequence == 4) {
                    timer1.startTimer();
                    sequence = 0;
                }
            }
            shoot3x();
            follower.update();
        }
    }

    private void three_close_shots() {
        LL.set_angle_close();
        startCloseShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();  // COMMENTED OUT (depo)
            if (depo.reachedTarget()) {  // COMMENTED OUT (depo)
                if (sequence == 3 || sequence == 4) {
                    timer1.startTimer();
                    sequence = 0;
                }
            }
            shoot3x();
            follower.update();
        }
    }

    private void first_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        // path 1
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, firstpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        // path 2 (forward hop)
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 23) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        follower.setMaxPower(0.5);
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        follower.setMaxPower(1.0);
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)
    }

    private void second_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(startPose, midPoint1, secondpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 25.5) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)
    }

    private void third_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        Pose cur = follower.getPose();
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint3, thirdpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), thirdpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur1 = follower.getPose();
        double heading = cur1.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 19) * Math.sin(heading);
        Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);

        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur1, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)
    }

    private void go_close() {
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint2, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }
    private void go_close_2() {
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }

    private boolean isFarShotCycleDone() {
        return (sequence == 0 && timer1.timerIsOff());
    }

    // unified shooting timing (copied from far-blue)
    private void shoot3x(){
        if(timer1.checkAtSeconds(0)){
            LL.leftUp();
        }
        if(timer1.checkAtSeconds(0.3)){
            LL.leftDown();
            LL.rightUp();
        }
        if(timer1.checkAtSeconds(0.6)){
            LL.rightDown();
            LL.backUp();
        }
        if(timer1.checkAtSeconds(1.1)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
        }

    }
}