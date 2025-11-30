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


//hi
@Autonomous(name = "newBot_closeRed", group = "Pedro")
public class newBot_closeRed extends LinearOpMode {

    // ---------- Shooter subsystems ----------
    private Deposition depo;
    private lift_three LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower;

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose startPose = new Pose(
            110,                // x inches
            -27,                     // y inches og:32
            Math.toRadians(-225)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose nearshotpose = new Pose(
            90,                    // x inches (forward) og: 72
            -8.5,                   // y inches (left)
            Math.toRadians(-223)    // heading (rad) at finish
    );
    private final Pose firstpickupPose = new Pose(
            66.5,                    // x inches (forward) og 71.5
            -6.5,                   // y inches (left) og: 22.5
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose secondpickupPose = new Pose(
            43.25,                    // x inches (forward) og 41.25
            -8,                   // y inches (left) og: 21
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose midpoint1     = new Pose(43.25,-2,  Math.toRadians(90.0));

    private final Pose thirdpickupPose = new Pose(
            20,                    // x inches (forward) og 16
            -9.5,                   // y inches (left) og: 22
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose homePose = new Pose(
            0.0,                    // x inches (forward)
            0.0,            // y inches (left)
            Math.toRadians(-25.0)    // heading (rad) at finish
    );

    private final Pose infront_of_lever   = new Pose(61.5, -37.5, Math.toRadians(0));



    private static final double SECOND_HOP_IN = 20.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)

    // ---------- Timing for far shots ----------
    private Timer timer1;
    private int sequence = 0;


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
        follower.setStartingPose(startPose);

        // Launcher safe start
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay) then run your movement.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        go_back();
        sleep(450);
        three_close_shots();
        first_line_pickup();
        reset();
        go_close();
        three_close_shots();
        second_line_pickup();
        reset();
        go_close();
        three_close_shots();
        third_line_pickup();
        reset();
        go_close();
        three_close_shots();
        go_infront();


        telemetry.addLine("ADITI WAD HEREEEEEEE");
        telemetry.update();
        sleep(500);

    }
    private void go_infront(){
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), homePose.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }
    private void go_back(){

        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);              // COMMENTED OUT (depo)

    }


    // ===== Far shot logic (exact from your teleOp) =====
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
    //ss
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


    private void first_line_pickup(){
        intake.setPower(-1);
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(nearshotpose, firstpickupPose)))
                .setLinearHeadingInterpolation(nearshotpose.getHeading(), firstpickupPose.getHeading(), 0.8)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+7.5) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);
        Path p2 = new Path(new BezierLine(cur, secondGoal));

        follower.setMaxPower(0.5);
        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(p2)
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        follower.setMaxPower(1);
    }
    private void second_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        intake.setPower(-1);
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(nearshotpose,secondpickupPose))) //add the midpoint
                .setLinearHeadingInterpolation(nearshotpose.getHeading(),secondpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+15) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }
    private void third_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        intake.setPower(-1);
        Pose cur = follower.getPose();
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, thirdpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(),thirdpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur1 = follower.getPose();
        double heading = cur1.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+15.5)* Math.sin(heading);
        Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);

        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur1, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void go_close(){
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1,nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(),nearshotpose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        intake.setPower(0);
    }

    private boolean isFarShotCycleDone() {
        return (sequence == 0 && timer1.timerIsOff());
    }
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