package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Turret pid test", group = "tuning")
@Config
public class TurretTest extends LinearOpMode {

    private DcMotorEx encoderMotor;   // Motor port used to read encoder

    private PIDController pid;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0.035;
    public static double i = 0.0;
    public static double d = 0.0;
    public static int target = 0;
    public static double tolerance = 1.0;

    private enum Mode { DRIVER, AUTO }
    private Mode mode = Mode.DRIVER;

    @Override
    public void runOpMode() {
        encoderMotor = hardwareMap.get(DcMotorEx.class, "turret");

        // Don’t actually drive the motor, just use its encoder
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p, i, d);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            pid.setPID(p, i, d);

            double currentPos = encoderMotor.getCurrentPosition();

            if (gamepad1.y) mode = Mode.AUTO;
            if (gamepad1.b) mode = Mode.DRIVER;

            double power;

            if (mode == Mode.DRIVER) {
                // Manual control
                power = -gamepad1.left_stick_y;
            } else {
                // PID control
                double error = target - currentPos;

                if (Math.abs(error) <= tolerance) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, target);
                    power = Math.max(-0.75, Math.min(0.75, pidOutput));
                }

            }

            encoderMotor.setPower(power);

            telemetry.addData("Mode", mode);
            telemetry.addData("Target", target);
            telemetry.addData("Encoder Position", currentPos);
            telemetry.addData("Error", target - currentPos);
            telemetry.addData("Servo Power", power);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.update();
        }
    }
}
