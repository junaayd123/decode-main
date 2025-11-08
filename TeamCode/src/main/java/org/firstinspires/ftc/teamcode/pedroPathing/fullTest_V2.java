package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "fullTest_V2", group = "TeleOp")
public class fullTest_V2 extends LinearOpMode {

    // --- Hardware ---
    private DcMotor intakeMotor;
    private CRServo sorterServo;
    private NormalizedColorSensor colorSensor;
    private DcMotor tbenc;

    // --- Config constants ---
    private static final double INTAKE_POWER = 1.0;
    private static final double SORTER_POWER = 0.13;
    private static final double COLOR_NO_BALL_THRESH = 0.005;
    private static final double TICKS_PER_REV = 8192.0;

    private static final int CHAMBER_A = 0;
    private static final int CHAMBER_B = 120;
    private static final int CHAMBER_C = 240;

    // --- Runtime state ---
    private boolean intakeRunning = false;
    private boolean sorterRunning = false;
    private boolean firstRotationDone = false;
    private double startPosition = 0;

    private Map<Character, Character> chamberColorMap = new HashMap<>();

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware mapping ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterServo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");

        tbenc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbenc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sorterServo.setPower(0.0);
        intakeMotor.setPower(0.0);

        // Initialize chambers as 'N'
        chamberColorMap.put('A', 'N');
        chamberColorMap.put('B', 'N');
        chamberColorMap.put('C', 'N');

        telemetry.clearAll();
        telemetry.addLine("=== Intake + Split Sorter System Ready ===");
        telemetry.addLine("Press A / X to start intake.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            int currentAngle = getCurrentAngle();
            NormalizedRGBA colors = readColorsSafely();
            boolean ballPresent = isBallPresent(colors);

            // --- Start intake ---
            if (!intakeRunning && (gamepad1.a || gamepad1.cross)) {
                intakeMotor.setPower(INTAKE_POWER);
                intakeRunning = true;
            }

            // --- Start sorter at first ball ---
            if (intakeRunning && !sorterRunning && ballPresent) {
                sorterRunning = true;
                startPosition = tbenc.getCurrentPosition();
            }

            if (sorterRunning) {
                double ticksMoved = Math.abs(tbenc.getCurrentPosition() - startPosition);
                double rotationsMoved = ticksMoved / TICKS_PER_REV;

                // Update chamber color if ball present
                if (ballPresent) {
                    char chamber = getNearestChamber(currentAngle);
                    char color = colorStateToLetter(colorStateFromColors(colors));
                    chamberColorMap.put(chamber, color);
                }

                // --- Handle split rotations ---
                if (!firstRotationDone && rotationsMoved >= 1.0) {
                    // Stop briefly
                    sorterServo.setPower(0.0);
                    sleep(300); // 0.3 seconds pause
                    // Reset start position for second rotation
                    startPosition = tbenc.getCurrentPosition();
                    firstRotationDone = true;
                    sorterServo.setPower(SORTER_POWER);
                }

                // Stop after second rotation
                if (firstRotationDone && rotationsMoved >= 1.0) {
                    sorterRunning = false;
                    sorterServo.setPower(0.0);
                    intakeMotor.setPower(0.0);
                    intakeRunning = false;

                    // Fill empty chambers with 'N' if none detected
                    if (!chamberColorMap.containsKey('A')) chamberColorMap.put('A', 'N');
                    if (!chamberColorMap.containsKey('B')) chamberColorMap.put('B', 'N');
                    if (!chamberColorMap.containsKey('C')) chamberColorMap.put('C', 'N');
                }

                // Ensure servo is running if sorterRunning
                if (sorterRunning) sorterServo.setPower(SORTER_POWER);
            }

            // --- Telemetry ---
            telemetry.addLine("--- Status ---");
            telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
            telemetry.addData("Sorter", sorterRunning ? "ON" : "OFF");
            telemetry.addData("Angle", currentAngle);
            telemetry.addLine("--- Chambers ---");
            telemetry.addData("A", chamberColorMap.get('A'));
            telemetry.addData("B", chamberColorMap.get('B'));
            telemetry.addData("C", chamberColorMap.get('C'));
            telemetry.update();

            idle();
        }

        sorterServo.setPower(0.0);
        intakeMotor.setPower(0.0);
    }

    // --- Helper methods ---
    private int getCurrentAngle() {
        double deg = -tbenc.getCurrentPosition() / (TICKS_PER_REV / 360.0);
        return Math.floorMod((int)Math.floor(deg), 360);
    }

    private NormalizedRGBA readColorsSafely() {
        try {
            return colorSensor.getNormalizedColors();
        } catch (Exception e) {
            return new NormalizedRGBA() {{
                red = 0f; green = 0f; blue = 0f; alpha = 0f;
            }};
        }
    }

    private boolean isBallPresent(NormalizedRGBA c) {
        return !(c.red < COLOR_NO_BALL_THRESH && c.green < COLOR_NO_BALL_THRESH && c.blue < COLOR_NO_BALL_THRESH);
    }

    private int colorStateFromColors(NormalizedRGBA c) {
        if (c.red < COLOR_NO_BALL_THRESH && c.green < COLOR_NO_BALL_THRESH && c.blue < COLOR_NO_BALL_THRESH) return 0;
        if (c.green > c.blue && c.green > c.red) return 1;
        if (c.blue >= c.green && c.blue > c.red) return 2;
        return 0;
    }

    private char colorStateToLetter(int state) {
        if (state == 1) return 'G';
        if (state == 2) return 'P';
        return 'N';
    }

    private char getNearestChamber(int currentAngle) {
        int da = angleDiffAbs(currentAngle, CHAMBER_A);
        int db = angleDiffAbs(currentAngle, CHAMBER_B);
        int dc = angleDiffAbs(currentAngle, CHAMBER_C);
        if (da <= db && da <= dc) return 'A';
        if (db <= da && db <= dc) return 'B';
        return 'C';
    }

    private int angleDiffAbs(double a, double b) {
        int diff = (int)Math.abs(a - b) % 360;
        return Math.min(diff, 360 - diff);
    }
}
