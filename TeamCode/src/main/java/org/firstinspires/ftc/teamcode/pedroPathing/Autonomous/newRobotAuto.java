package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
//
@Autonomous(name = "LM2 Square Test", group = "Testing")
public class newRobotAuto extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(90))); // Facing “up”

        // Build a square path: (0,0) -> (5,0) -> (5,5) -> (0,5) -> (0,0)
        /*Path path = follower.pathBuilder()
                .addPath(new PathBuilder(
                        .addBezier(new Pose(0, 0, Math.toRadians(90)), new Pose(5, 0))
                        .addBezier(new Pose(5, 0), new Pose(5, 5))
                        .addBezier(new Pose(5, 5), new Pose(0, 5))
                        .addBezier(new Pose(0, 5), new Pose(0, 0))
                        .build()
                )
                .build();

        waitForStart();

        if (isStopRequested()) return;*/

        //follower.followPath(path);

        // Keep updating follower while it drives
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }

        //follower.stop();
    }
}
