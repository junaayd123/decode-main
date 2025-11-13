package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;


@Autonomous(name = "redAuto", group = "Pedro")
public class redAuto extends LinearOpMode {

    // ---------- Shooter subsystems ----------
    private Deposition depo;
    private launch_lift LL;
    private DcMotor intake = null;
    private DcMotor d1 = null;
    private DcMotor d2 = null;
    // ---------- Pedro ----------
    private Follower follower;

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose startPose = new Pose(
            0.0,                // x inches
            0.0,                     // y inches
            Math.toRadians(-45.0)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose goalPose = new Pose(
            -42.0,                    // x inches (forward)
            42.0,                   // y inches (right)
            Math.toRadians(-45.0)    // heading (rad) at finish
    );
    private final Pose goal_1_Pose = new Pose(
            -43.0,                    // x inches (forward)
            -43.0,                   // y inches (right)
            Math.toRadians(-90.0)    // heading (rad) at finish
    );

    private static final double SECOND_HOP_IN = 13.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)

    // ---------- Timing for far shots ----------
    private final ElapsedTime timer = new ElapsedTime();
    private double curTime = 10000;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init subsystems
        depo = new Deposition(hardwareMap);
        LL   = new launch_lift(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        d1 = hardwareMap.get(DcMotor.class, "depo");
        d2 = hardwareMap.get(DcMotor.class, "depo1");

        if (d1   != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2   != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Launcher safe start
        LL.down();
        LL.close();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay) then run your movement.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset();

//
        move_back();
        three_shots();
        // reset();
        //go_home();
        //three_shots();
        //second_line_pickup();
        //reset();
        //go_home();
        //three_shots();




        telemetry.addLine("✅ Done: fired 3 shots (with delay) + completed both paths.");
        telemetry.update();
        sleep(500);

    }

    private void reset(){
        depo.setPowerBoth(0.0);
        LL.down();
        LL.close();
        intake.setPower(0);
    }

    private void go_home(){
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

    // ===== Far shot logic (exact from your teleOp) =====
    private void startFarShot() {
        curTime = timer.seconds();
        depo.shootFar();
        LL.far();
    }
    private void three_shots(){
        for (int i = 0; i < 3 && opModeIsActive(); i++) {

            if (i==0) {
                startFarShot();
            }
            else{
                intake.setPower(-1.0);
                startFarShot();
            }
            // Wait for this shot to finish
            while (opModeIsActive() && !isFarShotCycleDone()) {
                farcheck();          // keep timing logic same
                follower.update();
            }

            depo.setPowerBoth(0.0);  // safety: ensure shooter stops fully

            // 🔹 Delay between shots (let launcher reset)

        }
    }
    private void move_back(){
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, goalPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goalPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void first_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, goalPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goalPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = SECOND_HOP_IN * Math.sin(heading);
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
    private void second_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(startPose, goal_1_Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), goal_1_Pose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = SECOND_HOP_IN * Math.sin(heading);
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

    private boolean isFarShotCycleDone() {
        return (curTime == 10000);
    }

    public void farcheck(){
        if(timer.seconds() >= curTime + .4 && timer.seconds() < curTime + .5){
            LL.up();
        }
        if (timer.seconds() >= curTime + 1.1 && timer.seconds() < curTime + 1.2){
            LL.down();
        }
        if(timer.seconds() >= curTime + 1.7 && timer.seconds() < curTime + 1.8){
            depo.setPowerBoth(0.0);
        }
        if(timer.seconds()>=curTime+4.3 && timer.seconds()<curTime+4.4){

            curTime = 10000;
        }
    }
}