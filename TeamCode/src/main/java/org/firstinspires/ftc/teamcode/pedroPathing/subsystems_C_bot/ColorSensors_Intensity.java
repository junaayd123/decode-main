package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensors_Intensity {

    public NormalizedColorSensor SensorLeft;
    public NormalizedColorSensor SensorRight;
    public NormalizedColorSensor SensorBack;
    public NormalizedColorSensor SensorLeft2;
    public NormalizedColorSensor SensorRight2;
    public NormalizedColorSensor SensorBack2;

    // Raw floor below which readings are always considered empty.
    private static final float HARD_MIN_INTENSITY = 0.008f;
    // Intensity above ambient floor needed to count as a ball on one sensor.
    private static final float STRONG_ABOVE_FLOOR = 0.0065f;
    // If both sensors in a slot clear this, treat as ball present.
    private static final float WEAK_ABOVE_FLOOR = 0.0035f;
    // Only adapt ambient floor when very near current floor to avoid drifting into ball readings.
    private static final float FLOOR_ADAPT_MARGIN = 0.002f;
    // Slot stays occupied for this long after last seen to handle fast push-through events.
    private static final double STICKY_HOLD_SEC = 0.50;
    // If chamber was full recently, keep a short full latch for rapid push/rearrange events.
    private static final double FULL_LATCH_SEC = 0.70;

    private float floorLeft = 0.0f;
    private float floorRight = 0.0f;
    private float floorBack = 0.0f;
    private float floorLeft2 = 0.0f;
    private float floorRight2 = 0.0f;
    private float floorBack2 = 0.0f;

    private boolean rightRaw;
    private boolean backRaw;
    private boolean leftRaw;
    private boolean rightSticky;
    private boolean backSticky;
    private boolean leftSticky;
    private double lastRefreshSec = -1.0;

    private double lastSeenRightSec = -100.0;
    private double lastSeenBackSec = -100.0;
    private double lastSeenLeftSec = -100.0;
    private double lastSeenFullSec = -100.0;

    public ColorSensors_Intensity(HardwareMap hardwareMap) {
        SensorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back");
        SensorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        SensorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right");
        SensorBack2 = hardwareMap.get(NormalizedColorSensor.class, "color_back2");
        SensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_left2");
        SensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_right2");

        calibrateAmbientFloor();
    }

    public void update() {
        refreshStates();
    }

    public void calibrateAmbientFloor() {
        floorRight = readIntensity(SensorRight);
        floorRight2 = readIntensity(SensorRight2);
        floorBack = readIntensity(SensorBack);
        floorBack2 = readIntensity(SensorBack2);
        floorLeft = readIntensity(SensorLeft);
        floorLeft2 = readIntensity(SensorLeft2);

        rightRaw = false;
        backRaw = false;
        leftRaw = false;
        rightSticky = false;
        backSticky = false;
        leftSticky = false;

        lastSeenRightSec = -100.0;
        lastSeenBackSec = -100.0;
        lastSeenLeftSec = -100.0;
        lastSeenFullSec = -100.0;
        lastRefreshSec = -1.0;
    }

    public boolean detectBallByIntensity(NormalizedColorSensor sensor) {
        refreshStates();
        return detectBallInstant(sensor);
    }

    public float readIntensity(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        return colors.red + colors.green + colors.blue;
    }

    public boolean hasBall(NormalizedColorSensor sensorA, NormalizedColorSensor sensorB) {
        refreshStates();
        return detectBallInstant(sensorA) || detectBallInstant(sensorB);
    }

    // Sticky-by-default API: tuned for protecting against fast ball movement.
    public boolean rightHasBall() {
        refreshStates();
        return rightSticky;
    }

    public boolean backHasBall() {
        refreshStates();
        return backSticky;
    }

    public boolean leftHasBall() {
        refreshStates();
        return leftSticky;
    }

    // Raw (non-sticky) reads in case a caller wants immediate state only.
    public boolean rightHasBallRaw() {
        refreshStates();
        return rightRaw;
    }

    public boolean backHasBallRaw() {
        refreshStates();
        return backRaw;
    }

    public boolean leftHasBallRaw() {
        refreshStates();
        return leftRaw;
    }

    public boolean isFull() {
        return rightHasBall() && backHasBall() && leftHasBall();
    }

    public boolean isFullRaw() {
        refreshStates();
        return rightRaw && backRaw && leftRaw;
    }

    public boolean isLikelyFull() {
        refreshStates();
        double nowSec = System.nanoTime() * 1e-9;
        boolean anyNow = rightRaw || backRaw || leftRaw;
        boolean stickyFull = rightSticky && backSticky && leftSticky;
        boolean recentFull = (nowSec - lastSeenFullSec) <= FULL_LATCH_SEC;
        return stickyFull || (recentFull && anyNow);
    }

    // Slot format matches existing pattern: 0 = no ball, 1 = ball present.
    public int getRight() {
        return rightHasBall() ? 1 : 0;
    }

    public int getBack() {
        return backHasBall() ? 1 : 0;
    }

    public int getLeft() {
        return leftHasBall() ? 1 : 0;
    }

    public int getRightRaw() {
        return rightHasBallRaw() ? 1 : 0;
    }

    public int getBackRaw() {
        return backHasBallRaw() ? 1 : 0;
    }

    public int getLeftRaw() {
        return leftHasBallRaw() ? 1 : 0;
    }

    public String formatIntensityPair(NormalizedColorSensor sensorA, NormalizedColorSensor sensorB) {
        return String.format("%.4f / %.4f", readIntensity(sensorA), readIntensity(sensorB));
    }

    public double getStickyHoldSec() {
        return STICKY_HOLD_SEC;
    }

    public double getFullLatchSec() {
        return FULL_LATCH_SEC;
    }

    private void refreshStates() {
        double nowSec = System.nanoTime() * 1e-9;

        // Avoid recalculating multiple times in the same loop/instant.
        if (lastRefreshSec > 0 && nowSec - lastRefreshSec < 0.0015) {
            return;
        }
        lastRefreshSec = nowSec;

        float rightA = correctedIntensity(SensorRight, 0);
        float rightB = correctedIntensity(SensorRight2, 1);
        float backA = correctedIntensity(SensorBack, 2);
        float backB = correctedIntensity(SensorBack2, 3);
        float leftA = correctedIntensity(SensorLeft, 4);
        float leftB = correctedIntensity(SensorLeft2, 5);

        rightRaw = pairDetect(rightA, rightB);
        backRaw = pairDetect(backA, backB);
        leftRaw = pairDetect(leftA, leftB);

        if (rightRaw) lastSeenRightSec = nowSec;
        if (backRaw) lastSeenBackSec = nowSec;
        if (leftRaw) lastSeenLeftSec = nowSec;

        rightSticky = rightRaw || (nowSec - lastSeenRightSec <= STICKY_HOLD_SEC);
        backSticky = backRaw || (nowSec - lastSeenBackSec <= STICKY_HOLD_SEC);
        leftSticky = leftRaw || (nowSec - lastSeenLeftSec <= STICKY_HOLD_SEC);

        if ((rightRaw && backRaw && leftRaw) || (rightSticky && backSticky && leftSticky)) {
            lastSeenFullSec = nowSec;
        }
    }

    private boolean detectBallInstant(NormalizedColorSensor sensor) {
        int floorIndex = floorIndex(sensor);
        float corrected = correctedIntensity(sensor, floorIndex);
        return corrected >= STRONG_ABOVE_FLOOR;
    }

    private boolean pairDetect(float correctedA, float correctedB) {
        // Crosstalk-resistant rule:
        // - one strong sensor is enough, or
        // - both sensors modestly above ambient.
        return correctedA >= STRONG_ABOVE_FLOOR
                || correctedB >= STRONG_ABOVE_FLOOR
                || (correctedA >= WEAK_ABOVE_FLOOR && correctedB >= WEAK_ABOVE_FLOOR);
    }

    private float correctedIntensity(NormalizedColorSensor sensor, int idx) {
        float raw = readIntensity(sensor);
        float floor = getFloor(idx);

        // Adapt floor only near empty readings to reduce bleed-over effects.
        if (raw < HARD_MIN_INTENSITY || raw < floor + FLOOR_ADAPT_MARGIN) {
            floor = 0.9f * floor + 0.1f * raw;
            setFloor(idx, floor);
        }

        float corrected = raw - floor;
        return Math.max(0.0f, corrected);
    }

    private int floorIndex(NormalizedColorSensor sensor) {
        if (sensor == SensorRight) return 0;
        if (sensor == SensorRight2) return 1;
        if (sensor == SensorBack) return 2;
        if (sensor == SensorBack2) return 3;
        if (sensor == SensorLeft) return 4;
        return 5;
    }

    private float getFloor(int idx) {
        switch (idx) {
            case 0:
                return floorRight;
            case 1:
                return floorRight2;
            case 2:
                return floorBack;
            case 3:
                return floorBack2;
            case 4:
                return floorLeft;
            default:
                return floorLeft2;
        }
    }

    private void setFloor(int idx, float value) {
        switch (idx) {
            case 0:
                floorRight = value;
                break;
            case 1:
                floorRight2 = value;
                break;
            case 2:
                floorBack = value;
                break;
            case 3:
                floorBack2 = value;
                break;
            case 4:
                floorLeft = value;
                break;
            default:
                floorLeft2 = value;
                break;
        }
    }
}
