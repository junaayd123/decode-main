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

    private static final double POSITION_TOLERANCE = 2.0; // inches
    private static final double HEADING_TOLERANCE = Math.toRadians(5); // degrees



    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "rfmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "lfmotor");
        backRight = hardwareMap.get(DcMotor.class, "rbmotor");
        backLeft = hardwareMap.get(DcMotor.class, "lbmotor");
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        follower.activateAllPIDFs();
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.5, 0.07, 0.3, 0.1));
        follower.setSecondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.35, 0, 0.02, 0.025));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(3, 0.003, 0.09, 0.09));
        follower.setSecondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0.0005, 0.05, 0.01));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.035, 0.003, 0, 0.6, 0));
    }

    @Override
    public void start() {
        targetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());
        holding = true;
        hardwareBrake();
    }

    @Override
    public void loop() {
        follower.update();

        if (!holding) return;

        Pose current = follower.getPose();
        double posError = Math.hypot(
                current.getX() - targetPose.getX(),
                current.getY() - targetPose.getY()
        );
        double headingError = Math.abs(current.getHeading() - targetPose.getHeading());

        boolean outOfTolerance = posError > POSITION_TOLERANCE || headingError > HEADING_TOLERANCE;

        if (outOfTolerance) {
            // PID correct back to target
            follower.holdPoint(targetPose);
        } else {
            // Close enough — hand off to hardware brake
            follower.breakFollowing();
            hardwareBrake();
        }
    }

    private void hardwareBrake() {
        // Replace these with however your drivetrain motors are accessed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
