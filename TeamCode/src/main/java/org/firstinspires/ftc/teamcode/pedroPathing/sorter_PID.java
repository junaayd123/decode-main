package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// If PedroPathing has its own @Config annotation, import that instead.
// Otherwise, remove the @Config line below.
//import com.arcrobotics.ftclib.command.CommandOpMode; // optional, safe to ignore if unused

@TeleOp(name = "CRServo Position PID (Pedro)", group = "Tuning")
public class sorter_PID extends LinearOpMode {

    // --- Tunable PID Coefficients ---
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // --- Target Position ---
    public static double targetPosition = 0.0;

    // --- Position tolerance ---
    public static double positionTolerance = 5.0;

    private CRServo servo;
    private DcMotorEx encoder;
    private ElapsedTime timer = new ElapsedTime();

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    private boolean runToTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "sorter");
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");

        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("CRServo PID (Pedro) Ready");
        telemetry.update();

        waitForStart();
        timer.reset();
        lastTime = timer.seconds();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Control mode toggle
            if (gamepad1.a) {
                runToTarget = true;
            } else if (gamepad1.b) {
                runToTarget = false;
                servo.setPower(0);
            }

            double currentPosition = encoder.getCurrentPosition();
            double error = targetPosition - currentPosition;
            double servoPower = 0;

            if (runToTarget) {
                // PID computation
                if (Math.abs(error) > positionTolerance) {
                    integralSum += error * dt;
                } else {
                    integralSum = 0;
                }

                double derivative = (error - lastError) / dt;
                double output = (kP * error) + (kI * integralSum) + (kD * derivative);
                lastError = error;

                servoPower = Range.clip(output, -1.0, 1.0);

                if (Math.abs(error) < positionTolerance) {
                    servoPower = 0;
                }

                servo.setPower(servoPower);
            }

            telemetry.addData("Mode", runToTarget ? "RUNNING (A pressed)" : "STOPPED (B pressed)");
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.update();
        }
    }
}
