package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
//import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
@TeleOp(name = "auto localization", group = "TeleOp")

public class closeRedLocalizationTest extends OpMode {
    private final Pose startPose = new Pose(
            44,                // x inches
            128,                     // y inches og:32
            Math.toRadians(35)    // heading (rad)
    );
    private Follower follower;
    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        Pose cur = follower.getPose();
        telemetry.addData("x:" ,cur.getX());

        telemetry.addData("y:" , cur.getY());
        telemetry.addData("heading:" , cur.getHeading());
        telemetry.addData("total heading:" , follower.getTotalHeading());
        telemetry.update();

        drawCurrentAndHistory();
    }
}