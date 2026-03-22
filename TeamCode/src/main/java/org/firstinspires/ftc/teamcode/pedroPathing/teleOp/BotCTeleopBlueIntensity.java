package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ColorSensors_Intensity;

@TeleOp(name = "Bot C blue intake intensity test", group = "A_TeleOp")
public class BotCTeleopBlueIntensity extends OpMode {

    private ColorSensors_Intensity intensitySensors;
    private DcMotor intake;

    private boolean prevRightBumper = false;

    private boolean intakeRunning = false;
    private boolean outtakeActive = false;
    private double outtakeStartSec = -1.0;
    private double fullCandidateStartSec = -1.0;

    private boolean intRightRaw;
    private boolean intBackRaw;
    private boolean intLeftRaw;
    private boolean intRightSticky;
    private boolean intBackSticky;
    private boolean intLeftSticky;
    private boolean intLikelyFull;

    private static final double INTAKE_POWER = -1.0;
    private static final double OUTTAKE_POWER = 1.0;
    private static final double OUTTAKE_DURATION_SEC = 0.40;
    private static final double FULL_CONFIRM_SEC = 0.01;

    @Override
    public void init() {
        intensitySensors = new ColorSensors_Intensity(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        prevRightBumper = false;

        telemetry.addLine("Bot C intensity intake test ready");
        telemetry.addLine("RB: intake on/off | LB: manual outtake");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        intensitySensors.calibrateAmbientFloor();
        intakeRunning = false;
        outtakeActive = false;
        outtakeStartSec = -1.0;
        fullCandidateStartSec = -1.0;
        intake.setPower(0);
    }

    @Override
    public void loop() {
        updateBallStates();

        boolean rbPressed = gamepad2.right_bumper && !prevRightBumper;

        if (rbPressed) {
            if (intakeRunning || outtakeActive) {
                intakeRunning = false;
                stopOuttake();
                intake.setPower(0);
            } else {
                // On toggle-on, protect against already-full chamber.
                if (shouldOuttakeOnToggleOn()) {
                    intakeRunning = false;
                    startOuttake(getRuntime());
                } else {
                    intakeRunning = true;
                }
            }
        }

        if (gamepad2.left_bumper) {
            intakeRunning = false;
            stopOuttake();
            intake.setPower(OUTTAKE_POWER);
        } else {
            runIntakeAndAutoOuttake(getRuntime());
        }

        telemetry.addData("Mode", "INTENSITY_ONLY");
        telemetry.addData("Intake Toggle", intakeRunning);
        telemetry.addData("Intake Running", intake.getPower() < -0.05);
        telemetry.addData("Outtaking", intake.getPower() > 0.05);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Outtake Active", outtakeActive);
        telemetry.addData("Intensity Full Raw", isFullByIntensityRaw());
        telemetry.addData("Intensity Full Sticky", isFullByIntensitySticky());
        telemetry.addData("Intensity Likely Full", intLikelyFull);
        telemetry.addData("Guardian Full", isFullByGuardian());
        telemetry.addData("Toggle-On Full", shouldOuttakeOnToggleOn());
        telemetry.addData("Full Candidate", isFullByGuardian() || isFullByIntensityCandidate());
        telemetry.addData("Full Confirming", fullCandidateStartSec >= 0 ? "YES" : "NO");

        telemetry.addData("INT Raw Ball Slot", "R:%s B:%s L:%s",
                intRightRaw,
                intBackRaw,
                intLeftRaw);
        telemetry.addData("INT Sticky Slot", "R:%s B:%s L:%s",
                intRightSticky,
                intBackSticky,
                intLeftSticky);

        telemetry.addData("Right Intensity", intensitySensors.formatIntensityPair(intensitySensors.SensorRight, intensitySensors.SensorRight2));
        telemetry.addData("Back Intensity", intensitySensors.formatIntensityPair(intensitySensors.SensorBack, intensitySensors.SensorBack2));
        telemetry.addData("Left Intensity", intensitySensors.formatIntensityPair(intensitySensors.SensorLeft, intensitySensors.SensorLeft2));
        telemetry.addData("Sticky Hold Sec", intensitySensors.getStickyHoldSec());
        telemetry.addData("Full Latch Sec", intensitySensors.getFullLatchSec());
        telemetry.update();

        prevRightBumper = gamepad2.right_bumper;
    }

    private void runIntakeAndAutoOuttake(double nowSec) {
        if (outtakeActive) {
            if (nowSec - outtakeStartSec <= OUTTAKE_DURATION_SEC) {
                intake.setPower(OUTTAKE_POWER);
            } else {
                stopOuttake();
                fullCandidateStartSec = -1.0;
                intake.setPower(0);
            }
            return;
        }

        if (!intakeRunning) {
            fullCandidateStartSec = -1.0;
            intake.setPower(0);
            return;
        }

        boolean fullCandidate = isFullByGuardian() || isFullByIntensityCandidate();
        if (fullCandidate) {
            if (fullCandidateStartSec < 0) {
                fullCandidateStartSec = nowSec;
            }

            if (nowSec - fullCandidateStartSec >= FULL_CONFIRM_SEC) {
                intakeRunning = false;
                startOuttake(nowSec);
            }
        } else {
            fullCandidateStartSec = -1.0;
            intake.setPower(INTAKE_POWER);
        }
    }

    private void startOuttake(double nowSec) {
        outtakeActive = true;
        outtakeStartSec = nowSec;
        intake.setPower(OUTTAKE_POWER);
    }

    private void stopOuttake() {
        outtakeActive = false;
        outtakeStartSec = -1.0;
    }

    private boolean isFullByIntensityCandidate() {
        return isFullByIntensityRaw() || intLikelyFull;
    }

    private boolean isFullByGuardian() {
        // Guard using sticky + require at least 2 current slots occupied.
        int nowCount = 0;
        if (isRightNow()) nowCount++;
        if (isBackNow()) nowCount++;
        if (isLeftNow()) nowCount++;

        // Allow fast push/rearrange cases where 4th ball can temporarily displace
        // two balls from instantaneous view.
        return intLikelyFull && nowCount >= 1;
    }

    private boolean shouldOuttakeOnToggleOn() {
        int nowCount = 0;
        if (isRightNow()) nowCount++;
        if (isBackNow()) nowCount++;
        if (isLeftNow()) nowCount++;

        // Strong immediate check for "already full" when enabling intake.
        return isFullByIntensityRaw()
                || (isFullByIntensitySticky() && nowCount >= 2)
                || (intLikelyFull && nowCount >= 2);
    }

    private boolean isRightNow() {
        return intRightRaw;
    }

    private boolean isBackNow() {
        return intBackRaw;
    }

    private boolean isLeftNow() {
        return intLeftRaw;
    }

    private boolean isFullByIntensityRaw() {
        return intRightRaw && intBackRaw && intLeftRaw;
    }

    private boolean isFullByIntensitySticky() {
        return intRightSticky && intBackSticky && intLeftSticky;
    }

    private void updateBallStates() {
        intensitySensors.update();

        intRightRaw = intensitySensors.rightHasBallRaw();
        intBackRaw = intensitySensors.backHasBallRaw();
        intLeftRaw = intensitySensors.leftHasBallRaw();

        intRightSticky = intensitySensors.rightHasBall();
        intBackSticky = intensitySensors.backHasBall();
        intLeftSticky = intensitySensors.leftHasBall();
        intLikelyFull = intensitySensors.isLikelyFull();
    }
}
