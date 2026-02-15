package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;

@TeleOp(name = "Freeze Drive Test", group="z")
public class freezeDriveTest extends OpMode {

    public Follower follower;

    private final Pose startPose = new Pose(
            44,
            128,
            Math.toRadians(35)
    );

    // --- Freeze state ---
    private boolean frozen = false;
    private Pose holdPose;

    // button edge detection
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;

        // ---- FREEZE ----
        if (aPressed && !frozen) {
            frozen = true;

            // Save current pose
            holdPose = follower.getPose();

            // Tell follower to hold this pose
            follower.holdPoint(holdPose);

        }

        // ---- UNFREEZE ----
        if (bPressed && frozen) {
            frozen = false;

            // Resume teleop driving
            follower.startTeleopDrive();
        }

        // ---- CONTROL LOGIC ----
        if (!frozen) {
            follower.setTeleOpDrive( -gamepad1.left_stick_y,
                    (gamepad1.left_trigger - gamepad1.right_trigger),
                    -gamepad1.right_stick_x, true );
            // Normal teleop drive
        }
        // If frozen: DO NOT setTeleOpDrive
        // follower will fight to stay at holdPose automatically

        follower.update();

        telemetry.addData("Mode", frozen ? "FROZEN (HOLDING POSE)" : "TELEOP");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }
}
