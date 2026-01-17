package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Obsolete;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
@Disabled
//
@Autonomous(name = "farRedMotif_PATHING_ONLY", group = "Pedro")
public class farRedMotif_PATHING extends LinearOpMode {

    // ---------- Pedro ----------
    private Follower follower;

    // ---------- Poses ----------
    private final Pose start_align_Pose = new Pose(-4.0, 2, Math.toRadians(-180));
    private final Pose startPose        = new Pose(0.0, 0.0, Math.toRadians(-201));

    private final Pose firstpickupPose  = new Pose(22.5, -20, Math.toRadians(-90));

    private final Pose midPoint1        = new Pose(37, -14, Math.toRadians(-90));
    private final Pose secondpickupPose = new Pose(45.5, -17, Math.toRadians(-90));

    private final Pose midPoint2        = new Pose(44, -4, Math.toRadians(-90));
    private final Pose thirdpickupPose  = new Pose(71.5, -20, Math.toRadians(-90));

    private final Pose midPoint3        = new Pose(76, -4, Math.toRadians(-90));
    private final Pose near_shot_Pose   = new Pose(97.5, -17, Math.toRadians(-237));

    private final Pose infront_of_lever = new Pose(61.5, -37.5, Math.toRadians(-180));

    private static final double SECOND_HOP_IN = 13.5;

    @Override
    public void runOpMode() {

        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(start_align_Pose);

        telemetry.addLine("✅ Pathing-only auto ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        first_align_movement();
        first_line_pickup();
        go_home();

        second_line_pickup();
        go_home();

        third_line_pickup();
        go_close();
        go_infront();

        telemetry.addLine("✅ Finished pathing-only auto");
        telemetry.update();
        sleep(500);
    }

    // ---------------- PATH METHODS ----------------

    private void first_align_movement() {
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(start_align_Pose, startPose)))
                .setLinearHeadingInterpolation(
                        start_align_Pose.getHeading(),
                        startPose.getHeading()
                )
                .build();

        follow(path);
    }

    private void go_home() {
        Pose cur = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, startPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), startPose.getHeading())
                .build();

        follow(path);
    }

    private void go_infront() {
        Pose cur = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever.getHeading())
                .build();

        follow(path);
    }

    private void go_close() {
        Pose cur = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint2, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();

        follow(path);
    }

    private void first_line_pickup() {
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, firstpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstpickupPose.getHeading())
                .build();

        follow(path);
        forwardHop(SECOND_HOP_IN, 18);
    }

    private void second_line_pickup() {
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(startPose, midPoint1, secondpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondpickupPose.getHeading())
                .build();

        follow(path);
        forwardHop(SECOND_HOP_IN, 22);
    }

    private void third_line_pickup() {
        Pose cur = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint3, thirdpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), thirdpickupPose.getHeading())
                .build();

        follow(path);
        forwardHop(SECOND_HOP_IN, 13.5);
    }

    // ---------------- HELPERS ----------------

    private void forwardHop(double forward, double extra) {
        Pose cur = follower.getPose();
        double h = cur.getHeading();

        Pose goal = new Pose(
                cur.getX() + forward * Math.cos(h),
                cur.getY() + (forward + extra) * Math.sin(h),
                h
        );

        PathChain hop = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, goal)))
                .setConstantHeadingInterpolation(h)
                .setTimeoutConstraint(0.2)
                .build();

        follow(hop);
    }

    private void follow(PathChain path) {
        follower.followPath(path, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }
}
