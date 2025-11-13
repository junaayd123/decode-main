package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Depo Velocity PID Tuner", group = "tuning")
@Config
public class shooterPID_test extends LinearOpMode {

    private DcMotorEx depo;   // Has encoder
    private DcMotor depo1;    // Follower, no encoder

    private PIDController pid;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // PID coefficients (tune these in dashboard)
    public static double p = -0.001;
    public static double i = 0.0;
    public static double d = 0.00;

    // Target velocity in ticks per second
    public static double targetVelocity = 0;

    // Optional: motor-specific constant for simple feedforward
    public static double kF = -0.00048; // adjust for your motor’s max ticks/sec

    // Mode switching
    private enum Mode { DRIVER, AUTO }
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        depo = hardwareMap.get(DcMotorEx.class, "depo");
        depo1 = hardwareMap.get(DcMotor.class, "depo1");

        depo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        depo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(p, i, d);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Allow live PID tuning
            pid.setPID(p, i, d);

            // Read current velocity (ticks per second)
            double currentVelocity = depo.getVelocity();

            // --- Gamepad control ---
            if (gamepad1.y) mode = Mode.AUTO;   // enable PID control
            if (gamepad1.b) mode = Mode.DRIVER; // return to manual mode

            double power;

            if (mode == Mode.DRIVER) {
                power = 0;
                // Manual control: joystick sets power directly
                depo.setPower(gamepad1.left_stick_y);
                depo1.setPower(gamepad1.right_stick_y);

                // Optional: adjust target velocity dynamically for testing
                if (gamepad1.dpad_up) targetVelocity += 100;
                if (gamepad1.dpad_down) targetVelocity -= 100;
            } else {
                // PID velocity control
                double pidOutput = pid.calculate(currentVelocity, targetVelocity);

                // Simple feedforward term to help overcome friction
                double ff = kF * targetVelocity;

                power = pidOutput + ff;
                power = Math.max(-1, Math.min(1, power)); // Clamp to safe range
                depo.setPower(power);
                depo1.setPower(power);
            }

            // Apply to both motors (follower mirrors depo)


            telemetry.addData("Mode", mode);
            telemetry.addData("Target Velocity (ticks/s)", targetVelocity);
            telemetry.addData("Current Velocity (ticks/s)", currentVelocity);
            telemetry.addData("Error", targetVelocity - currentVelocity);
            telemetry.addData("Power Output", power);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.addData("kF", kF);
            telemetry.update();
        }
    }
}
