package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous
public class TBEncoderTesting extends LinearOpMode {
    private CRServo sorterServo;
    private NormalizedColorSensor colorSensor;
    private float red, green, blue;
    private int state;

    private DcMotor tbenc;

    // Control constants
    private final double kP = 0.02;          // proportional gain
    private final double minPower = 0.08;    // minimum to overcome friction
    private final double maxPower = 0.7;     // max servo power
    private final int deadband = 5;          // tolerance in degrees
    private final int slowRange = 60;        // start slowing down when within this range
    private final int chamberOffset = 60;    // offset to align chamber properly

    private int targetAngle = -1;            // target chamber

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterservo");
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");

        tbenc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbenc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // --- Current angle ---
            int degrees = (int) Math.floor((double) -tbenc.getCurrentPosition() / (8192.0 / 360.0));
            int currentAngle = Math.floorMod(degrees, 360);

            // --- Read color ---
            try {
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                red = colors.red;
                green = colors.green;
                blue = colors.blue;
            } catch (Exception e) {
                telemetry.addData("ColorSensor Error", e);
            }

            // --- Determine color state ---
            if (red < 0.005 && green < 0.005 && blue < 0.005) state = 0;
            else if (green > blue && green > red) state = 1;
            else if (blue >= green && blue > red) state = 2;

            // --- Button mapping ---
            if (gamepad1.x) targetAngle = 0 + chamberOffset;   // chamber 1
            if (gamepad1.a) targetAngle = 120 + chamberOffset; // chamber 2
            if (gamepad1.b) targetAngle = 240 + chamberOffset; // chamber 3
            if (gamepad1.y) targetAngle = -1; // stop

            // wrap target angle to 0..360
            if (targetAngle >= 0) targetAngle = targetAngle % 360;

            double power = 0;

            if (targetAngle >= 0) {
                // --- Compute error ---
                int error = targetAngle - currentAngle;

                // wrap -180..180
                if (error > 180) error -= 360;
                if (error < -180) error += 360;

                int absError = Math.abs(error);

                // Only correct if outside deadband
                if (absError > deadband) {

                    // Non-linear easing: squared scale
                    double scale = 1.0;
                    if (absError < slowRange) {
                        scale = Math.pow((double) absError / slowRange, 2);
                    }

                    power = kP * error * scale;

                    // enforce minimum to overcome friction
                    if (power > 0 && power < minPower) power = minPower;
                    if (power < 0 && power > -minPower) power = -minPower;

                    // clamp max
                    if (power > maxPower) power = maxPower;
                    if (power < -maxPower) power = -maxPower;
                } else {
                    power = 0; // inside deadband, stop
                }
            }

            sorterServo.setPower(power);

            // --- Telemetry ---
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", targetAngle >= 0 ? targetAngle - currentAngle : 0);
            telemetry.addData("Servo Power", String.format("%.2f", power));
            telemetry.addData("Color State", state);
            telemetry.update();
        }
    }
}
