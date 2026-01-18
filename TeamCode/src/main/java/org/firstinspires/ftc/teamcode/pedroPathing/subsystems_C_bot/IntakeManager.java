package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

public class IntakeManager {

    private DcMotorEx intake;
    private ColorSensors_New sensors;
    private Timer reverseTimer;
    private Timer jamCheckTimer;
    private Timer startupIgnoreTimer; // ✅ NEW: Ignore current spikes during motor startup

    private enum IntakeState {
        IDLE,
        COLLECTING,
        REVERSING,
        JAM_RECOVERY
    }

    private IntakeState currentState = IntakeState.IDLE;
    private int consecutiveFullReadings = 0;
    private int consecutiveJamReadings = 0;
    private int consecutiveHighCurrentReadings = 0;

    private double currentDraw = 0;
    private double peakCurrent = 0;
    private double averageCurrent = 0;
    private double currentSum = 0;
    private int currentSampleCount = 0;
    private boolean motorOverloaded = false;

    private boolean highCurrentTimerActive = false; // ✅ Track if high current timer is running

    // Configuration constants
    private static final int FULL_INTAKE_COUNT = 3;
    private static final double REVERSE_DURATION = 0.5;
    private static final double JAM_CHECK_INTERVAL = 0.15;
    private static final int JAM_THRESHOLD = 2;
    private static final int FULL_THRESHOLD = 2;

    // ✅ SMART CURRENT MONITORING: Ignores startup inrush, detects sustained high current
    private static final double STARTUP_IGNORE_TIME = 0.15; // Ignore first 150ms of startup (reduced from 300ms)
    private static final double SUSTAINED_HIGH_CURRENT = 3.5; // Amps - jam if sustained
    private static final double SUSTAINED_TIME_THRESHOLD = 0.1; // Must be high for 100ms (reduced from 150ms)
    private static final double NORMAL_RUNNING_CURRENT = 2.2; // Expected during normal operation

    private Timer highCurrentTimer; // ✅ Tracks how long current has been high

    public IntakeManager(HardwareMap hardwareMap, ColorSensors_New colorSensors) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sensors = colorSensors;
        reverseTimer = new Timer();
        jamCheckTimer = new Timer();
        startupIgnoreTimer = new Timer();
        highCurrentTimer = new Timer();
        highCurrentTimerActive = false;
    }

    public void update() {
        updateCurrentReading();

        // ✅ SMART JAM DETECTION: Only check after startup period
        if (currentState == IntakeState.COLLECTING) {
            if (startupIgnoreTimer.checkAtSeconds(STARTUP_IGNORE_TIME)) {
                checkForSustainedHighCurrent();
            }
        }

        if (jamCheckTimer.checkAtSeconds(JAM_CHECK_INTERVAL)) {
            checkForJamsAndOverflow();
            jamCheckTimer.resetTimer();
        }

        switch (currentState) {
            case IDLE:
                currentSum = 0;
                currentSampleCount = 0;
                peakCurrent = 0;
                break;

            case COLLECTING:
                if (isIntakeFull()) {
                    consecutiveFullReadings++;
                    if (consecutiveFullReadings >= FULL_THRESHOLD) {
                        startReverse();
                    }
                } else {
                    consecutiveFullReadings = 0;
                }
                break;

            case REVERSING:
                if (reverseTimer.checkAtSeconds(REVERSE_DURATION)) {
                    stopReverse();
                }
                break;

            case JAM_RECOVERY:
                if (reverseTimer.checkAtSeconds(REVERSE_DURATION * 2)) {
                    stopJamRecovery();
                }
                break;
        }
    }

    private void updateCurrentReading() {
        try {
            currentDraw = intake.getCurrent(CurrentUnit.AMPS);

            // Track statistics (only during normal operation, not startup)
            if (currentState == IntakeState.COLLECTING &&
                    startupIgnoreTimer.checkAtSeconds(STARTUP_IGNORE_TIME)) {
                currentSum += currentDraw;
                currentSampleCount++;
                averageCurrent = currentSum / currentSampleCount;

                if (currentDraw > peakCurrent) {
                    peakCurrent = currentDraw;
                }
            }
        } catch (Exception e) {
            currentDraw = 0;
        }
    }

    /**
     * ✅ SMART JAM DETECTION: Only triggers if current stays high for sustained period
     * This prevents false positives from startup inrush and brief spikes
     */
    private void checkForSustainedHighCurrent() {
        if (currentDraw > SUSTAINED_HIGH_CURRENT) {
            // Current is high - start or continue timing
            if (!highCurrentTimerActive) {
                highCurrentTimer.startTimer();
                highCurrentTimerActive = true;
            }

            // Check if it's been high for long enough
            if (highCurrentTimer.checkAtSeconds(SUSTAINED_TIME_THRESHOLD)) {
                motorOverloaded = true;
                startJamRecovery();
            }
        } else {
            // Current dropped back to normal - reset timer
            if (highCurrentTimerActive) {
                highCurrentTimer.stopTimer();
                highCurrentTimerActive = false;
            }
            motorOverloaded = false;
        }
    }

    public void startCollecting() {
        if (currentState != IntakeState.REVERSING && currentState != IntakeState.JAM_RECOVERY) {
            intake.setPower(-1);
            currentState = IntakeState.COLLECTING;
            consecutiveFullReadings = 0;
            consecutiveHighCurrentReadings = 0;
            jamCheckTimer.startTimer();
            startupIgnoreTimer.startTimer(); // ✅ Start ignoring startup current

            currentSum = 0;
            currentSampleCount = 0;
            peakCurrent = 0;
            averageCurrent = 0;
            motorOverloaded = false;

            // Reset high current timer
            if (highCurrentTimerActive) {
                highCurrentTimer.stopTimer();
                highCurrentTimerActive = false;
            }
        }
    }

    public void stop() {
        if (currentState == IntakeState.COLLECTING) {
            intake.setPower(0);
            currentState = IntakeState.IDLE;
            jamCheckTimer.stopTimer();
            startupIgnoreTimer.stopTimer();
            consecutiveHighCurrentReadings = 0;
            motorOverloaded = false;

            if (highCurrentTimerActive) {
                highCurrentTimer.stopTimer();
                highCurrentTimerActive = false;
            }
        }
    }

    public void manualReverse() {
        intake.setPower(1);
        currentState = IntakeState.IDLE;
        startupIgnoreTimer.stopTimer();

        if (highCurrentTimerActive) {
            highCurrentTimer.stopTimer();
            highCurrentTimerActive = false;
        }
    }

    public void manualStop() {
        intake.setPower(0);
        currentState = IntakeState.IDLE;
        jamCheckTimer.stopTimer();
        startupIgnoreTimer.stopTimer();
        consecutiveHighCurrentReadings = 0;
        motorOverloaded = false;

        if (highCurrentTimerActive) {
            highCurrentTimer.stopTimer();
            highCurrentTimerActive = false;
        }
    }

    private boolean isIntakeFull() {
        int ballCount = countBalls();
        return ballCount >= FULL_INTAKE_COUNT;
    }

    private int countBalls() {
        int count = 0;
        if (sensors.getLeft() != 0) count++;
        if (sensors.getRight() != 0) count++;
        if (sensors.getBack() != 0) count++;
        return count;
    }

    private void checkForJamsAndOverflow() {
        boolean potentialJam =
                sensors.isPotentialJam("left") ||
                        sensors.isPotentialJam("right") ||
                        sensors.isPotentialJam("back");

        boolean overflowDetected = isIntakeFull();

        if (potentialJam || overflowDetected) {
            consecutiveJamReadings++;
            if (consecutiveJamReadings >= JAM_THRESHOLD) {
                startJamRecovery();
            }
        } else {
            consecutiveJamReadings = 0;
        }
    }

    private void startReverse() {
        intake.setPower(1);
        currentState = IntakeState.REVERSING;
        reverseTimer.startTimer();
        consecutiveFullReadings = 0;
        consecutiveHighCurrentReadings = 0;
        motorOverloaded = false;

        startupIgnoreTimer.stopTimer();
        if (highCurrentTimerActive) {
            highCurrentTimer.stopTimer();
            highCurrentTimerActive = false;
        }
    }

    private void stopReverse() {
        intake.setPower(0);
        currentState = IntakeState.IDLE;
        reverseTimer.stopTimer();
        consecutiveJamReadings = 0;
    }

    private void startJamRecovery() {
        intake.setPower(1);
        currentState = IntakeState.JAM_RECOVERY;
        reverseTimer.startTimer();
        consecutiveJamReadings = 0;
        consecutiveHighCurrentReadings = 0;
        motorOverloaded = false;

        startupIgnoreTimer.stopTimer();
        if (highCurrentTimerActive) {
            highCurrentTimer.stopTimer();
            highCurrentTimerActive = false;
        }
    }

    private void stopJamRecovery() {
        intake.setPower(0);
        currentState = IntakeState.IDLE;
        reverseTimer.stopTimer();
    }

    public String getStateString() {
        String state = currentState.toString();
        if (motorOverloaded && currentState == IntakeState.COLLECTING) {
            state += " (HIGH CURRENT!)";
        }

        // ✅ Show startup ignore status
        if (currentState == IntakeState.COLLECTING &&
                !startupIgnoreTimer.checkAtSeconds(STARTUP_IGNORE_TIME)) {
            state += " [STARTUP]";
        }

        return state;
    }

    public int getBallCount() {
        return countBalls();
    }

    public boolean isReversing() {
        return currentState == IntakeState.REVERSING || currentState == IntakeState.JAM_RECOVERY;
    }

    public boolean isCollecting() {
        return currentState == IntakeState.COLLECTING;
    }

    public double getCurrentDraw() {
        return currentDraw;
    }

    public double getPeakCurrent() {
        return peakCurrent;
    }

    public double getAverageCurrent() {
        return averageCurrent;
    }

    public boolean isMotorOverloaded() {
        return motorOverloaded;
    }

    /**
     * ✅ NEW: Get how long current has been sustained high (for debugging)
     */
    public double getHighCurrentDuration() {
        if (highCurrentTimerActive) {
            return highCurrentTimer.timer.seconds() - highCurrentTimer.curtime;
        }
        return 0;
    }
}