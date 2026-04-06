package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Raw Heading Lock Drive", group = "z")
public class headingLockDriveTest extends OpMode {

    public static double HEADING_P                = -4.0;
    public static double HEADING_I                = -0.3;
    public static double HEADING_D                = -0.3;
    public static double TURN_DEADBAND            = 0.05;
    public static double TRANSLATION_DEADBAND     = 0.05;
    // integral only accumulates when error is below this (radians) — tightened to avoid windup on fwd/back
    public static double INTEGRAL_MAX_ERR         = 0.05;
    // seconds after turn stick release before locking heading
    public static double TURN_SETTLE_TIME         = 0.25;
    // seconds after translation stick release before locking heading — lets robot coast to a stop
    public static double TRANSLATION_SETTLE_TIME  = 0.25;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private GoBildaPinpointDriver pinpoint;

    private double  lockedHeading    = 0;
    private double  lastHeadingError = 0;
    private double  integralSum      = 0;
    private boolean wasTurning       = false;
    private boolean wasTranslating   = false;
    private boolean turningSettling  = false;
    private boolean transSettling    = false;

    private ElapsedTime loopTimer        = new ElapsedTime();
    private ElapsedTime settleTimer      = new ElapsedTime();
    private ElapsedTime transSettleTimer = new ElapsedTime();
    private MultipleTelemetry telemetryA;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "lfmotor");
        leftRear   = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfmotor");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rbmotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        pinpoint.update();
        lockedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);

        double speed   = gamepad1.cross ? 0.3 : 1.0;
        double forward = -gamepad1.left_stick_y * speed;
        double strafe  = (gamepad1.right_trigger - gamepad1.left_trigger) * speed;
        double turn    = gamepad1.right_stick_x * speed;

        boolean driverTurning     = Math.abs(gamepad1.right_stick_x) > TURN_DEADBAND;
        boolean driverTranslating = Math.abs(gamepad1.left_stick_y) > TRANSLATION_DEADBAND
                || Math.abs(gamepad1.right_trigger) > TRANSLATION_DEADBAND
                || Math.abs(gamepad1.left_trigger)  > TRANSLATION_DEADBAND;

        // if driver starts translating again during a translation settle, cancel the settle
        if (transSettling && driverTranslating) {
            transSettling = false;
        }

        double rotationOutput;
        String state;

        if (driverTurning) {
            // actively turning — pass stick through directly
            rotationOutput   = turn;
            wasTurning       = true;
            turningSettling  = false;
            transSettling    = false;
            lockedHeading    = heading;
            lastHeadingError = 0;
            integralSum      = 0;
            state = "TURNING";

        } else if (wasTurning) {
            // turn stick just released — start turn settle window
            wasTurning      = false;
            turningSettling = true;
            settleTimer.reset();
            rotationOutput  = 0;
            state = "TURN_SETTLING";

        } else if (turningSettling) {
            rotationOutput = 0;
            if (settleTimer.seconds() >= TURN_SETTLE_TIME) {
                lockedHeading    = heading;
                integralSum      = 0;
                lastHeadingError = 0;
                turningSettling  = false;
            }
            state = "TURN_SETTLING";

        } else if (wasTranslating && !driverTranslating) {
            // translation sticks just released — start translation settle window
            transSettling = true;
            transSettleTimer.reset();
            rotationOutput = 0;
            state = "TRANS_SETTLING";

        } else if (transSettling) {
            rotationOutput = 0;
            if (transSettleTimer.seconds() >= TRANSLATION_SETTLE_TIME) {
                // robot has coasted to a stop — snap lock to current heading and begin PID
                lockedHeading    = heading;
                integralSum      = 0;
                lastHeadingError = 0;
                transSettling    = false;
            }
            state = "TRANS_SETTLING";

        } else {
            // locked — run PID correction
            double headingError = lockedHeading - heading;
            while (headingError >  Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // integral only accumulates very close to target to prevent windup during fwd/back driving
            if (Math.abs(headingError) < INTEGRAL_MAX_ERR) {
                integralSum += headingError * dt;
            } else {
                integralSum = 0;
            }

            double derivative = dt > 0 ? (headingError - lastHeadingError) / dt : 0;
            lastHeadingError  = headingError;

            rotationOutput = headingError * HEADING_P
                    + integralSum  * HEADING_I
                    + derivative   * HEADING_D;
            state = "LOCKED";
        }

        wasTranslating = driverTranslating;

        // robot-centric mecanum mixing
        double lf = forward + strafe + rotationOutput;
        double lr = forward - strafe + rotationOutput;
        double rf = forward - strafe - rotationOutput;
        double rr = forward + strafe - rotationOutput;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                Math.max(Math.abs(rf), Math.abs(rr))));
        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);

        double headingError = lockedHeading - heading;
        while (headingError >  Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        telemetryA.addData("state",                   state);
        telemetryA.addData("heading (deg)",            String.format("%.2f", Math.toDegrees(heading)));
        telemetryA.addData("locked heading (deg)",     String.format("%.2f", Math.toDegrees(lockedHeading)));
        telemetryA.addData("heading error (deg)",      String.format("%.2f", Math.toDegrees(headingError)));
        telemetryA.addData("integral sum",             String.format("%.4f", integralSum));
        telemetryA.addData("rotation output",          String.format("%.3f", rotationOutput));
        telemetryA.addData("turn settle timer (s)",    String.format("%.2f", settleTimer.seconds()));
        telemetryA.addData("trans settle timer (s)",   String.format("%.2f", transSettleTimer.seconds()));
        telemetryA.addData("driver translating",       driverTranslating);
        telemetryA.addData("HEADING_P",                HEADING_P);
        telemetryA.addData("HEADING_I",                HEADING_I);
        telemetryA.addData("HEADING_D",                HEADING_D);
        telemetryA.addData("TURN_SETTLE_TIME",         TURN_SETTLE_TIME);
        telemetryA.addData("TRANSLATION_SETTLE_TIME",  TRANSLATION_SETTLE_TIME);
        telemetryA.addData("TRANSLATION_DEADBAND",     TRANSLATION_DEADBAND);
        telemetryA.addData("INTEGRAL_MAX_ERR (deg)",   String.format("%.1f", Math.toDegrees(INTEGRAL_MAX_ERR)));
        telemetryA.update();
    }
}