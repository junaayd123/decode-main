package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "fullTest", group = "TeleOp")
public class fullTest extends LinearOpMode {

    // hardware
    private DcMotor intakeMotor;
    private CRServo sorterServo;
    private NormalizedColorSensor colorSensor;
    private DcMotor tbenc;

    // config constants
    private final double INTAKE_POWER = 1.0;
    private final double SORTER_RUN_POWER = 0.1;
    private final double COLOR_NO_BALL_THRESH = 0.005; // same as your color code
    private final int ANGLE_PER_ROTATION_ENCODER = 8192; // through-bore encoder counts per revolution
    private final int angleOffset = 60; // your 60 degree offset
    private final int CHAMBER_SEPARATION = 120; // 3 chambers, 360/3 = 120

    // runtime state
    private int chamberZero = 0;
    private boolean sorterRunning = false;
    private boolean intakeRunning = false;

    // detection bookkeeping
    private boolean lastBallPresent = false;
    private Map<Character, Character> chamberColorMap = new HashMap<>(); // key: 'A'/'B'/'C' -> value 'P'/'G'/'N'
    private ArrayList<Character> detectionOrder = new ArrayList<>(); // P/G/N in order detected

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware map
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterservo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");

        // encoder setup
        tbenc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbenc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.clearAll();
        telemetry.addLine("=== Combined Intake+Sorter ===");
        telemetry.addLine("Align chamber exit to calibration position");
        telemetry.addLine("Press A (gamepad1.a) to calibrate chamber zero");
        telemetry.update();

        // wait for user to calibrate chamber zero before starting opmode
        while (!gamepad1.a && opModeIsActive()) {
            idle();
        }

        // calculate chamberZero from encoder (same math you used in TBEncoderTesting)
        int degrees = (int) Math.floor((double) -tbenc.getCurrentPosition() / (ANGLE_PER_ROTATION_ENCODER / 360.0));
        chamberZero = Math.floorMod(degrees, 360);
        telemetry.addData("Calibrated chamberZero (deg)", chamberZero);
        telemetry.addLine("Calibration complete. Press PLAY to run.");
        telemetry.update();

        waitForStart();

        // main loop
        while (opModeIsActive() && !isStopRequested()) {

            // read current encoder angle
            int deg = (int) Math.floor((double) -tbenc.getCurrentPosition() / (ANGLE_PER_ROTATION_ENCODER / 360.0));
            int currentAngle = Math.floorMod(deg, 360);

            // read color
            int colorState = readColorState(); // 0 = none, 1 = green, 2 = purple
            boolean ballPresent = (colorState != 0);

            // Buttons mapping
            boolean intakeOn     = gamepad1.cross || gamepad1.a;       // PS4 X / Logitech A
            boolean intakeOff    = gamepad1.circle || gamepad1.b;      // PS4 O / Logitech B
            boolean manualSorterOn  = gamepad1.square || gamepad1.x;   // PS4 Square / Logitech X
            boolean manualSorterOff = gamepad1.triangle || gamepad1.y; // PS4 Triangle / Logitech Y
            boolean runTogether  = gamepad1.ps || gamepad1.guide;      // PS4 PS / Logitech Guide

            // manual controls override / simple controls
            if (runTogether) {
                intakeMotor.setPower(INTAKE_POWER);
                sorterServo.setPower(SORTER_RUN_POWER);
                intakeRunning = true;
                sorterRunning = true;
            } else if (intakeOn) {
                intakeMotor.setPower(INTAKE_POWER);
                intakeRunning = true;
            } else if (intakeOff) {
                intakeMotor.setPower(0.0);
                intakeRunning = false;
            } else if (manualSorterOn) {
                sorterServo.setPower(SORTER_RUN_POWER);
                sorterRunning = true;
            } else if (manualSorterOff) {
                sorterServo.setPower(0.0);
                sorterRunning = false;
            }

            // Automatic behavior:
            // - When intake is running and the sensor sees a new ball (rising edge), start the sorter (if not already)
            // - On each rising-edge detection record color + chamber (A/B/C)
            // - Keep sorter running while collecting until we've filled all three chambers (A,B,C), then stop both
            if (intakeRunning) {
                // rising edge detection
                if (!lastBallPresent && ballPresent) {
                    // new ball detected
                    char colorLetter = colorStateToLetter(colorState); // 'P','G','N'
                    char chamberLetter = getNearestChamber(currentAngle); // 'A','B','C'

                    // Store only first value seen for a chamber (so we don't overwrite)
                    if (!chamberColorMap.containsKey(chamberLetter)) {
                        chamberColorMap.put(chamberLetter, colorLetter);
                        detectionOrder.add(colorLetter);

                        telemetry.addData("Detected", "Ball detected - Color:%s Chamber:%s", colorLetter, chamberLetter);
                    } else {
                        telemetry.addData("Detected", "Ball at chamber %s already recorded (%s) - ignoring", chamberLetter, chamberColorMap.get(chamberLetter));
                    }

                    // ensure sorter is running to move this ball into its chamber if not running
                    sorterServo.setPower(SORTER_RUN_POWER);
                    sorterRunning = true;
                }
            }

            // If the sorter is running and we've recorded all three chambers, stop motors
            if (sorterRunning && chamberColorMap.size() >= 3) {
                sorterServo.setPower(0.0);
                intakeMotor.setPower(0.0);
                sorterRunning = false;
                intakeRunning = false;
                telemetry.addLine("All three chambers recorded - stopping intake & sorter");
            }

            // Save lastBallPresent for edge detection
            lastBallPresent = ballPresent;

            // Telemetry summary: show chamber mapping and detection order
            telemetry.addLine("--- Status ---");
            telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
            telemetry.addData("Sorter", sorterRunning ? "ON (power " + SORTER_RUN_POWER + ")" : "OFF");
            telemetry.addData("Encoder Angle", currentAngle);
            telemetry.addData("ChamberZero", chamberZero);
            telemetry.addData("ColorSensor State", colorState + " (" + colorStateToString(colorState) + ")");
            telemetry.addLine("--- Chamber Colors ---");
            telemetry.addData("A", chamberColorMap.containsKey('A') ? chamberColorMap.get('A') : "-");
            telemetry.addData("B", chamberColorMap.containsKey('B') ? chamberColorMap.get('B') : "-");
            telemetry.addData("C", chamberColorMap.containsKey('C') ? chamberColorMap.get('C') : "-");
            telemetry.addLine("--- Detection Order ---");
            telemetry.addData("Order", detectionOrderToString());
            telemetry.update();

            idle(); // be cooperative
        }
    }

    // Convert colorState (0/1/2) to single letter
    private char colorStateToLetter(int state) {
        if (state == 1) return 'G';
        if (state == 2) return 'P';
        return 'N';
    }

    private String colorStateToString(int state) {
        if (state == 1) return "Green";
        if (state == 2) return "Purple";
        return "No Ball";
    }

    // Read normalized color and return 0 = none, 1 = green, 2 = purple
    private int readColorState() {
        try {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;

            if (red < COLOR_NO_BALL_THRESH && green < COLOR_NO_BALL_THRESH && blue < COLOR_NO_BALL_THRESH) {
                return 0;
            } else if (green > blue && green > red) {
                return 1;
            } else if (blue >= green && blue > red) {
                return 2;
            } else {
                return 0;
            }
        } catch (Exception e) {
            // If sensor reading fails, return 0 (no ball)
            return 0;
        }
    }

    // Determine nearest chamber (A,B,C) given current angle, using chamberZero + angleOffset + {0,120,240}
    private char getNearestChamber(int currentAngle) {
        int aAngle = Math.floorMod(chamberZero + angleOffset + 0, 360);
        int bAngle = Math.floorMod(chamberZero + angleOffset + CHAMBER_SEPARATION, 360);
        int cAngle = Math.floorMod(chamberZero + angleOffset + CHAMBER_SEPARATION * 2, 360);

        int da = angleDiffAbs(currentAngle, aAngle);
        int db = angleDiffAbs(currentAngle, bAngle);
        int dc = angleDiffAbs(currentAngle, cAngle);

        if (da <= db && da <= dc) return 'A';
        if (db <= da && db <= dc) return 'B';
        return 'C';
    }

    // absolute minimal difference between two angles (0..180)
    private int angleDiffAbs(int a, int b) {
        int diff = Math.abs(a - b) % 360;
        return Math.min(diff, 360 - diff);
    }

    // nice formatted detection order (e.g. P,G,N)
    private String detectionOrderToString() {
        if (detectionOrder.isEmpty()) return "-";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < detectionOrder.size(); i++) {
            sb.append(detectionOrder.get(i));
            if (i < detectionOrder.size() - 1) sb.append(",");
        }
        return sb.toString();
    }
}
