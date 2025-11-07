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
    private final double SORTER_RUN_POWER = 0.13;
    private final double COLOR_NO_BALL_THRESH = 0.005;
    private final int ANGLE_PER_ROTATION_ENCODER = 8192;
    private final int CHAMBER_DEADBAND = 10; // degrees tolerance for final alignment

    // chamber angles (deg)
    private final int CHAMBER_A = 0;
    private final int CHAMBER_B = 120;
    private final int CHAMBER_C = 240;

    // runtime state
    private boolean intakeRunning = false;
    private boolean sorterRunning = false;
    private boolean finishedSorting = false;
    private boolean rotatingToA = false;

    // detection bookkeeping
    private boolean lastBallPresent = false;
    private boolean firstBallHandled = false;      // used so intake pauses only on first ball
    private Map<Character, Character> chamberColorMap = new HashMap<>(); // 'A'/'B'/'C' -> 'G'/'P'/'N'
    private ArrayList<Character> detectionOrder = new ArrayList<>();     // order e.g. P,G,N

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware map (match names in your configuration)
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        sorterServo = hardwareMap.get(CRServo.class, "sorterservo");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");

        // encoder init (keep relative behavior)
        tbenc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tbenc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.clearAll();
        telemetry.addLine("=== Combined Intake+Sorter (fixed) ===");
        telemetry.addLine("Align chamber A to exit before starting.");
        telemetry.addLine("Press A / Cross to start intake, B/Circle to stop.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // current angle from encoder (0..359)
            int deg = (int) Math.floor((double) -tbenc.getCurrentPosition() / (ANGLE_PER_ROTATION_ENCODER / 360.0));
            int currentAngle = Math.floorMod(deg, 360);

            // read color sensor
            NormalizedRGBA colors = readColorsSafely();
            boolean ballPresent = isBallPresent(colors);

            // buttons
            boolean intakeOn = gamepad1.a || gamepad1.cross;
            boolean intakeOff = gamepad1.b || gamepad1.circle;

            // manual intake control
            if (intakeOn && !intakeRunning && !finishedSorting) {
                intakeMotor.setPower(INTAKE_POWER);
                intakeRunning = true;
                // sorter remains OFF until first detection
            } else if (intakeOff && intakeRunning) {
                intakeMotor.setPower(0.0);
                intakeRunning = false;
            }

            // automatic detection while intake running and not finished
            if (intakeRunning && !finishedSorting) {
                // rising edge: no ball -> ball
                if (!lastBallPresent && ballPresent) {
                    int colorState = colorStateFromColors(colors); // 0 = none, 1 = green, 2 = purple
                    char colorLetter = colorStateToLetter(colorState);
                    char chamberLetter = getNearestChamber(currentAngle);

                    // store only if chamber not recorded yet
                    if (!chamberColorMap.containsKey(chamberLetter)) {
                        chamberColorMap.put(chamberLetter, colorLetter);
                        detectionOrder.add(colorLetter);
                        telemetry.addData("Detected", "Color:%s Chamber:%s", colorLetter, chamberLetter);
                    }

                    // Start sorter immediately on detection (first or subsequent)
                    sorterServo.setPower(SORTER_RUN_POWER);
                    sorterRunning = true;

                    // If this is the first ball handled, pause intake for 1.0s
                    if (!firstBallHandled) {
                        firstBallHandled = true;
                        intakeMotor.setPower(0.0);      // stop intake
                        sleep(1000);                   // blocking pause - desired 1 second
                        // resume intake unless we've already finished (race check)
                        if (!finishedSorting) {
                            intakeMotor.setPower(INTAKE_POWER);
                        }
                    }
                }
            }

            // update lastSeen
            lastBallPresent = ballPresent;

            // After three unique chambers recorded -> finish sequence
            if (!finishedSorting && chamberColorMap.size() >= 3) {
                finishedSorting = true;

                // keep sorter spinning a little longer (0.2s), then stop and rotate to A
                sorterServo.setPower(SORTER_RUN_POWER);
                sleep(200);
                sorterServo.setPower(0);
                intakeMotor.setPower(0.0);
                intakeRunning = false;
                sorterRunning = false;

                telemetry.addLine("All 3 chambers recorded - rotating to Chamber A");
                telemetry.update();

                rotatingToA = true;
            }

            // Rotate to Chamber A (gentle proportional) until within deadband, then stop
            if (rotatingToA) {
                int error = getAngleError(currentAngle, CHAMBER_A);
                if (Math.abs(error) <= CHAMBER_DEADBAND) {
                    sorterServo.setPower(0.0);
                    rotatingToA = false;
                    telemetry.addLine("Aligned to Chamber A - stopped");
                } else {
                    double power = 0.003 * error; // small P gain
                    power = clamp(power, -0.18, 0.18);
                    sorterServo.setPower(power);
                }
            }

            // telemetry
            telemetry.addLine("--- Status ---");
            telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
            telemetry.addData("Sorter", sorterRunning ? "ON" : "OFF");
            telemetry.addData("Angle", currentAngle);
            telemetry.addLine("--- Chambers ---");
            telemetry.addData("A", chamberColorMap.containsKey('A') ? chamberColorMap.get('A') : "-");
            telemetry.addData("B", chamberColorMap.containsKey('B') ? chamberColorMap.get('B') : "-");
            telemetry.addData("C", chamberColorMap.containsKey('C') ? chamberColorMap.get('C') : "-");
            telemetry.addLine("--- Order ---");
            telemetry.addData("Order", detectionOrderToString());
            telemetry.update();

            idle();
        }

        // ensure motors off on exit
        sorterServo.setPower(0.0);
        intakeMotor.setPower(0.0);
    }

    // ---------- Helpers ----------

    private NormalizedRGBA readColorsSafely() {
        try {
            return colorSensor.getNormalizedColors();
        } catch (Exception e) {
            // create a dummy "no ball" value using a helper class
            return new NormalizedRGBA() {{
                red = 0f; green = 0f; blue = 0f; alpha = 0f;
            }};
        }
    }

    private boolean isBallPresent(NormalizedRGBA colors) {
        return !(colors.red < COLOR_NO_BALL_THRESH && colors.green < COLOR_NO_BALL_THRESH && colors.blue < COLOR_NO_BALL_THRESH);
    }

    private int colorStateFromColors(NormalizedRGBA colors) {
        if (colors.red < COLOR_NO_BALL_THRESH && colors.green < COLOR_NO_BALL_THRESH && colors.blue < COLOR_NO_BALL_THRESH) return 0;
        if (colors.green > colors.blue && colors.green > colors.red) return 1;
        if (colors.blue >= colors.green && colors.blue > colors.red) return 2;
        return 0;
    }

    private char colorStateToLetter(int state) {
        if (state == 1) return 'G';
        if (state == 2) return 'P';
        return 'N';
    }

    // map encoder angle to nearest chamber letter
    private char getNearestChamber(int currentAngle) {
        int da = angleDiffAbs(currentAngle, CHAMBER_A);
        int db = angleDiffAbs(currentAngle, CHAMBER_B);
        int dc = angleDiffAbs(currentAngle, CHAMBER_C);

        if (da <= db && da <= dc) return 'A';
        if (db <= da && db <= dc) return 'B';
        return 'C';
    }

    private int getAngleError(int current, int target) {
        int error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    private int angleDiffAbs(int a, int b) {
        int diff = Math.abs(a - b) % 360;
        return Math.min(diff, 360 - diff);
    }

    private String detectionOrderToString() {
        if (detectionOrder.isEmpty()) return "-";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < detectionOrder.size(); i++) {
            sb.append(detectionOrder.get(i));
            if (i < detectionOrder.size() - 1) sb.append(",");
        }
        return sb.toString();
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
