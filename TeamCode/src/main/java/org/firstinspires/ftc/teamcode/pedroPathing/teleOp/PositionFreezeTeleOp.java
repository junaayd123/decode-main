package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;

/**
 * TeleOp that allows the robot to "freeze" its position using PID control.
 * It uses the Pedro Pathing follower for localization but applies powers to motors directly.
 * Press Cross (A) on Gamepad 1 to toggle the freeze.
 */
@Config
@TeleOp(name = "Position Freeze TeleOp", group = "TeleOp")
public class PositionFreezeTeleOp extends OpMode {
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public static double Xp=0.4;
    public static double Xi=0.0;
    public static double Xd=0.001;
    public static double Yp=0.4;
    public static double Yi=0.0;
    public static double Yd=0.001;
    public static double Hp=8.0;
    public static double Hi=0.0;
    public static double Hd=0.07;

    // PID Controllers for X, Y and Heading
    // Coefficients inspired by C_Bot_Constants translational and heading PIDFs
    private PIDController xPID = new PIDController(Xp, Xi, Xd);
    private PIDController yPID = new PIDController(Yp, Yi, Yd);
    private PIDController hPID = new PIDController(Hp, Hi, Hd);

    private boolean isFrozen = false;
    private Pose frozenPose;
    private boolean lastCross = false;

    @Override
    public void init() {
        // Initialize Dashboard Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Follower for localization
        follower = C_Bot_Constants.createFollower(hardwareMap);
        
        // Initialize Drivetrain Motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "lfmotor");
        leftRear   = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfmotor");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rbmotor");

        // Set Motor Directions (Matching Bot C configuration)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Enable Braking
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set an initial starting pose (0,0,0)
        follower.setStartingPose(new Pose(0, 0, 0));
        
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        // We don't call follower.startTeleopDrive() because we handle motor powers manually.
        // follower.update() will keep localization running.
    }

    @Override
    public void loop() {
        // Update PID coefficients from Dashboard
        xPID.setPID(Xp, Xi, Xd);
        yPID.setPID(Yp, Yi, Yd);
        hPID.setPID(Hp, Hi, Hd);

        // Update localization
        follower.update();
        Pose currentPose = follower.getPose();

        // Toggle freeze on Cross (A) press
        if (gamepad1.cross && !lastCross) {
            isFrozen = !isFrozen;
            if (isFrozen) {
                // Snap the current pose to hold
                frozenPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
                xPID.reset();
                yPID.reset();
                hPID.reset();
            }
        }
        lastCross = gamepad1.cross;

        double forward, strafe, turn;

        if (isFrozen) {
            // Calculate PID outputs in field-relative terms
            // calculate(measured, target)
            double xPowerField = xPID.calculate(currentPose.getX(), frozenPose.getX());
            double yPowerField = yPID.calculate(currentPose.getY(), frozenPose.getY());
            
            // Calculate heading error and normalize it to [-pi, pi]
            double hError = frozenPose.getHeading() - currentPose.getHeading();
            while (hError > Math.PI) hError -= 2 * Math.PI;
            while (hError < -Math.PI) hError += 2 * Math.PI;
            
            // Calculate heading power (using target as current + error)
            double hPower = hPID.calculate(currentPose.getHeading(), currentPose.getHeading() + hError);

            // Rotate field-centric powers to robot-centric coordinates
            // Pedro Pathing: X+ Forward, Y+ Left, Heading CCW
            double heading = currentPose.getHeading();
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            
            forward = xPowerField * cos + yPowerField * sin;
            // Robot Y+ is Left, but motor mapping +strafe is Right.
            // So strafe_motor = -robot_y = x_field * sin - y_field * cos
            strafe  = xPowerField * sin - yPowerField * cos;
            // Motor mapping +turn is Clockwise. hPower is positive for CCW.
            turn    = -hPower;
            
        } else {
            // Standard TeleOp manual control
            forward = -gamepad1.left_stick_y;
            strafe  = -gamepad1.left_stick_x;
            turn    = -gamepad1.right_stick_x;
        }

        // Apply powers to motors directly using Mecanum kinematics
        // Mapping used in BotCTeleop_HeadingLock:
        // FL = f + s + t
        // BL = f - s + t
        // FR = f - s - t
        // BR = f + s - t
        double lf = forward + strafe + turn;
        double lr = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rr = forward + strafe - turn;

        // Normalize motor powers to maintain ratios if exceeding 1.0
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                Math.max(Math.abs(rf), Math.abs(rr))));
        
        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);

        // Telemetry for debugging
        telemetry.addData("Mode", isFrozen ? "FROZEN" : "MANUAL");
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Heading", String.format("%.2f deg", Math.toDegrees(currentPose.getHeading())));
        
        if (isFrozen) {
            telemetry.addData("Target X", String.format("%.2f", frozenPose.getX()));
            telemetry.addData("Target Y", String.format("%.2f", frozenPose.getY()));
            telemetry.addData("Target Heading", String.format("%.2f deg", Math.toDegrees(frozenPose.getHeading())));
        }
        telemetry.update();
    }
}
