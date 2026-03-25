package org.firstinspires.ftc.teamcode.pedroPathing.Misc;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;

@TeleOp(name = "PEDRO BRAKE TEST NEW")
/*public class PedroBrakeTest extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.activateAllPIDFs();
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(1.0, 0.07, 0.3, 0.1));
        follower.setSecondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.02, 0.025));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(4.5, 0.003, 0.09, 0.09));
        follower.setSecondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0.0005, 0.05, 0.01));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0.003, 0, 0.6, 0));
    }

    @Override
    public void start() {
        follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()));
    }

    @Override
    public void loop() {
        follower.update();
    }
}*/
public class PedroBrakeTest extends OpMode {

    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    private Follower follower;
    private Pose targetPose;
    private boolean holding = false;

    private double drivePValue = 0.035;
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "rfmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "lfmotor");
        backRight = hardwareMap.get(DcMotor.class, "rbmotor");
        backLeft = hardwareMap.get(DcMotor.class, "lbmotor");
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.activateAllPIDFs();
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.8, 0.06, 0.4, 0.1));
        follower.setSecondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.35, 0, 0.02, 0.025));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(3, 0.003, 0.09, 0.09));
        follower.setSecondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0.0005, 0.05, 0.01));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.135, 0.003, 0, 0.6, 0));
    }

    @Override
    public void start() {
        // Replace these with however your drivetrain motors are accessed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        targetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        holding = true;
        follower.holdPoint(targetPose);
    }

    @Override
    public void loop() {
        follower.update();

        if (!holding) return;

        // Bumper control for Drive PIDF P value (step = 0.05)
        boolean curLeft = gamepad1.left_bumper;
        boolean curRight = gamepad1.right_bumper;

        if (curRight && !prevRightBumper) {
            drivePValue = Math.round((drivePValue + 0.05) * 1000.0) / 1000.0;
            follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(drivePValue, 0.003, 0, 0.6, 0));
        } else if (curLeft && !prevLeftBumper) {
            drivePValue = Math.max(0.0, Math.round((drivePValue - 0.05) * 1000.0) / 1000.0);
            follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(drivePValue, 0.003, 0, 0.6, 0));
        }
        prevLeftBumper = curLeft;
        prevRightBumper = curRight;

        telemetry.addData("Drive PIDF P", drivePValue);
        telemetry.update();
    }

}
