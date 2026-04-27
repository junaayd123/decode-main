//package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_Intensity;
//
//@TeleOp(name = "Bot C blue intake intensity", group = "A_TeleOp")
//public class BotCTeleopBlueIntensity extends OpMode {
//
//    private ColorSensors_Intensity intensitySensors;
//    private DcMotor intake;
//
//    private boolean intakeRunning   = false;
//    private boolean outtakeActive   = false;
//    private double  outtakeStartSec = -1.0;
//    private boolean waitingForClear = false;
//
//    // Button latch — true means "button was UP last loop, so next press is a fresh tap"
//    private boolean rbWasReleased = true;
//
//    // Raw & sticky sensor state
//    private boolean intRightRaw;
//    private boolean intBackRaw;
//    private boolean intLeftRaw;
//    private boolean intRightSticky;
//    private boolean intBackSticky;
//    private boolean intLeftSticky;
//    private boolean intLikelyFull;
//
//    // -------------------------------------------------------------------------
//    // TUNING CONSTANTS
//    // -------------------------------------------------------------------------
//    private static final double INTAKE_POWER         = -1.0;
//    private static final double OUTTAKE_POWER        =  1.0;
//    private static final double OUTTAKE_DURATION_SEC =  0.25;
//    // -------------------------------------------------------------------------
//
//    @Override
//    public void init() {
//        intensitySensors = new ColorSensors_Intensity(hardwareMap);
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addLine("Bot C intensity intake FAST ready");
//        telemetry.addLine("RB: intake on/off  |  LB: manual outtake");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        resetRuntime();
//        intensitySensors.calibrateAmbientFloor();
//        intakeRunning   = false;
//        outtakeActive   = false;
//        outtakeStartSec = -1.0;
//        waitingForClear = false;
//        rbWasReleased   = true;
//        intake.setPower(0);
//    }
//
//    @Override
//    public void loop() {
//        // 1. Read sensors.
//        updateBallStates();
//
//        // 2. Clear-gate: block re-trigger until chamber physically empties.
//        if (waitingForClear && allSensorsClear()) {
//            waitingForClear = false;
//        }
//
//        // 3. FAST PATH auto-outtake — fires the loop the 3rd ball is detected.
//        if (intakeRunning && !outtakeActive && !waitingForClear && isFullByIntensityRaw()) {
//            intakeRunning = false;
//            startOuttake(getRuntime());
//        }
//
//        // 4. Right-bumper toggle — latch pattern so a single tap always registers.
//        //    rbWasReleased ensures we only act on the FIRST loop the button is down,
//        //    no matter how short or long the press is.
//        boolean rb = gamepad2.right_bumper;
//        if (rb && rbWasReleased) {
//            // Rising edge — treat this as one tap regardless of hold duration.
//            rbWasReleased = false;
//
//            if (intakeRunning || outtakeActive) {
//                intakeRunning   = false;
//                waitingForClear = false;
//                stopOuttake();
//                intake.setPower(0);
//            } else {
//                if (shouldOuttakeOnToggleOn()) {
//                    startOuttake(getRuntime());
//                } else {
//                    intakeRunning   = true;
//                    waitingForClear = false;
//                }
//            }
//        } else if (!rb) {
//            // Button is up — arm the latch so the next press registers instantly.
//            rbWasReleased = true;
//        }
//
//        // 5. Manual outtake (left bumper).
//        if (gamepad2.left_bumper) {
//            intakeRunning   = false;
//            waitingForClear = false;
//            stopOuttake();
//            intake.setPower(OUTTAKE_POWER);
//        } else {
//            applyMotorPower(getRuntime());
//        }
//
//        // 6. Telemetry.
//        telemetry.addData("Mode",              "INTENSITY_FAST");
//        telemetry.addData("Intake Running",    intakeRunning);
//        telemetry.addData("Outtake Active",    outtakeActive);
//        telemetry.addData("Waiting For Clear", waitingForClear);
//        telemetry.addData("Intake Power",      intake.getPower());
//        telemetry.addData("Full RAW",          isFullByIntensityRaw());
//        telemetry.addData("Full Sticky",       isFullByIntensitySticky());
//        telemetry.addData("Likely Full",       intLikelyFull);
//        telemetry.addData("All Clear",         allSensorsClear());
//        telemetry.addData("RB Was Released",   rbWasReleased);
//        telemetry.addData("INT Raw  R|B|L",    "%s | %s | %s", intRightRaw,    intBackRaw,    intLeftRaw);
//        telemetry.addData("INT Sticky R|B|L",  "%s | %s | %s", intRightSticky, intBackSticky, intLeftSticky);
//        telemetry.addData("Right Intensity",   intensitySensors.formatIntensityPair(intensitySensors.SensorRight, intensitySensors.SensorRight2));
//        telemetry.addData("Back Intensity",    intensitySensors.formatIntensityPair(intensitySensors.SensorBack,  intensitySensors.SensorBack2));
//        telemetry.addData("Left Intensity",    intensitySensors.formatIntensityPair(intensitySensors.SensorLeft,  intensitySensors.SensorLeft2));
//        telemetry.addData("Sticky Hold Sec",   intensitySensors.getStickyHoldSec());
//        telemetry.addData("Full Latch Sec",    intensitySensors.getFullLatchSec());
//        telemetry.update();
//    }
//
//    // -------------------------------------------------------------------------
//    // Motor control
//    // -------------------------------------------------------------------------
//
//    private void applyMotorPower(double nowSec) {
//        if (outtakeActive) {
//            if (nowSec - outtakeStartSec <= OUTTAKE_DURATION_SEC) {
//                intake.setPower(OUTTAKE_POWER);
//            } else {
//                stopOuttake();
//                waitingForClear = true;
//                intake.setPower(intakeRunning ? INTAKE_POWER : 0);
//            }
//            return;
//        }
//        intake.setPower(intakeRunning ? INTAKE_POWER : 0);
//    }
//
//    private void startOuttake(double nowSec) {
//        outtakeActive   = true;
//        outtakeStartSec = nowSec;
//        intake.setPower(OUTTAKE_POWER);
//    }
//
//    private void stopOuttake() {
//        outtakeActive   = false;
//        outtakeStartSec = -1.0;
//    }
//
//    // -------------------------------------------------------------------------
//    // Detection helpers
//    // -------------------------------------------------------------------------
//
//    private boolean allSensorsClear() {
//        return !intRightRaw && !intBackRaw && !intLeftRaw;
//    }
//
//    private boolean isFullByIntensityRaw() {
//        return intRightRaw && intBackRaw && intLeftRaw;
//    }
//
//    private boolean isFullByIntensitySticky() {
//        return intRightSticky && intBackSticky && intLeftSticky;
//    }
//
//    private boolean shouldOuttakeOnToggleOn() {
//        int nowCount = 0;
//        if (intRightRaw) nowCount++;
//        if (intBackRaw)  nowCount++;
//        if (intLeftRaw)  nowCount++;
//
//        return isFullByIntensityRaw()
//                || (isFullByIntensitySticky() && nowCount >= 2)
//                || (intLikelyFull            && nowCount >= 2);
//    }
//
//    private void updateBallStates() {
//        intensitySensors.update();
//
//        intRightRaw    = intensitySensors.rightHasBallRaw();
//        intBackRaw     = intensitySensors.backHasBallRaw();
//        intLeftRaw     = intensitySensors.leftHasBallRaw();
//
//        intRightSticky = intensitySensors.rightHasBall();
//        intBackSticky  = intensitySensors.backHasBall();
//        intLeftSticky  = intensitySensors.leftHasBall();
//        intLikelyFull  = intensitySensors.isLikelyFull();
//    }
//}