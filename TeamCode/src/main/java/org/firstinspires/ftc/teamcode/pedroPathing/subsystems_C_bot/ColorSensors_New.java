package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensors_New {

    public NormalizedColorSensor SensorLeft;
    public NormalizedColorSensor SensorRight;
    public NormalizedColorSensor SensorBack;
    public NormalizedColorSensor SensorLeft2;
    public NormalizedColorSensor SensorRight2;
    public NormalizedColorSensor SensorBack2;

    // Updated thresholds based on comprehensive data analysis
    private static final float MIN_INTENSITY = 0.015f; // Minimum total intensity to consider valid

    // Key detection thresholds from data:
    // - Green balls: b/g always < 0.7
    // - Purple balls: b/g typically > 1.0 (can be 0.85-1.27 when pushed/jammed)
    // - No ball: b/g around 0.8-0.95 AND b/r around 0.98-1.1
    private static final float BLUE_GREEN_RATIO_GREEN_MAX = 0.70f; // Green always < 0.7
    private static final float BLUE_GREEN_RATIO_PURPLE_MIN = 1.00f; // Purple typically > 1.0
    private static final float BLUE_GREEN_RATIO_NO_BALL_MIN = 0.78f; // No ball range
    private static final float BLUE_GREEN_RATIO_NO_BALL_MAX = 0.96f; // No ball range

    // Blue/Red ratio for no ball detection (b/r ~1.0 indicates no ball)
    private static final float BLUE_RED_RATIO_NO_BALL_MIN = 0.95f;
    private static final float BLUE_RED_RATIO_NO_BALL_MAX = 1.15f;

    public ColorSensors_New(HardwareMap hardwareMap) {
        SensorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back");
        SensorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        SensorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right");
        SensorBack2 = hardwareMap.get(NormalizedColorSensor.class, "color_back2");
        SensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_left2");
        SensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_right2");
    }

    public int getColor(NormalizedColorSensor colorSensor){
        // 0 = no ball, 1 = green, 2 = purple

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        // Calculate total intensity
        float totalIntensity = red + green + blue;

        // --- Step 1: Detect if no ball is present based on very low intensity ---
        if (totalIntensity < MIN_INTENSITY) {
            return 0; // No ball - too dim
        }

        // Calculate key ratios
        float blueGreenRatio = (green > 0.001f) ? (blue / green) : 0f;
        float blueRedRatio = (red > 0.001f) ? (blue / red) : 0f;

        // --- Step 2: Check for "no ball" using characteristic ratio pattern ---
        // No ball shows: b/g around 0.8-0.95 AND b/r around 0.98-1.1
        if (blueGreenRatio >= BLUE_GREEN_RATIO_NO_BALL_MIN &&
                blueGreenRatio <= BLUE_GREEN_RATIO_NO_BALL_MAX &&
                blueRedRatio >= BLUE_RED_RATIO_NO_BALL_MIN &&
                blueRedRatio <= BLUE_RED_RATIO_NO_BALL_MAX) {
            return 0; // No ball - matches "no ball" signature
        }

        // --- Step 3: Detect Green vs Purple ---
        // Green balls: b/g ratio consistently < 0.7 (even when pushed/jammed)
        if (blueGreenRatio < BLUE_GREEN_RATIO_GREEN_MAX) {
            return 1; // Green ball
        }
        // Purple balls: b/g ratio > 1.0 typically (can go as low as 0.85 when pushed)
        // We use 1.0 as threshold since no-ball is filtered out above
        else if (blueGreenRatio >= BLUE_GREEN_RATIO_PURPLE_MIN) {
            return 2; // Purple ball
        }
        // Ambiguous zone (0.7 - 1.0) - likely purple being pushed or transitional
        else {
            // In the 0.7-1.0 range, check b/r ratio to disambiguate
            // If b/r is far from 1.0, it's likely a ball (probably purple)
            if (blueRedRatio < 0.9 || blueRedRatio > 1.2) {
                return 2; // Likely purple (pushed/jammed state)
            } else {
                // Close to no-ball signature, but b/g is off - uncertain
                // Default to no ball to avoid false positives
                return 0;
            }
        }
    }

    public int getRight(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorRight);
        verdict2 = getColor(SensorRight2);

        // Both sensors agree on color
        if (verdict1 == verdict2 && verdict1 != 0) {
            return verdict1;
        }

        // One sensor sees a ball, other doesn't - trust the one that sees a ball
        if (verdict1 != 0 && verdict2 == 0) {
            return verdict1;
        }
        if (verdict2 != 0 && verdict1 == 0) {
            return verdict2;
        }

        // Sensors disagree on color - this might indicate multiple balls
        // Prioritize non-zero readings
        if (verdict1 != 0) {
            return verdict1;
        }
        if (verdict2 != 0) {
            return verdict2;
        }

        return 0; // Both see no ball
    }

    public int getLeft(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorLeft);
        verdict2 = getColor(SensorLeft2);

        if (verdict1 == verdict2 && verdict1 != 0) {
            return verdict1;
        }

        if (verdict1 != 0 && verdict2 == 0) {
            return verdict1;
        }
        if (verdict2 != 0 && verdict1 == 0) {
            return verdict2;
        }

        if (verdict1 != 0) {
            return verdict1;
        }
        if (verdict2 != 0) {
            return verdict2;
        }

        return 0;
    }

    public int getBack(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorBack);
        verdict2 = getColor(SensorBack2);

        if (verdict1 == verdict2 && verdict1 != 0) {
            return verdict1;
        }

        if (verdict1 != 0 && verdict2 == 0) {
            return verdict1;
        }
        if (verdict2 != 0 && verdict1 == 0) {
            return verdict2;
        }

        if (verdict1 != 0) {
            return verdict1;
        }
        if (verdict2 != 0) {
            return verdict2;
        }

        return 0;
    }

    // Helper method to detect potential jams based on sensor disagreement
    public boolean isPotentialJam(String position) {
        int verdict1 = 0, verdict2 = 0;

        switch(position) {
            case "right":
                verdict1 = getColor(SensorRight);
                verdict2 = getColor(SensorRight2);
                break;
            case "left":
                verdict1 = getColor(SensorLeft);
                verdict2 = getColor(SensorLeft2);
                break;
            case "back":
                verdict1 = getColor(SensorBack);
                verdict2 = getColor(SensorBack2);
                break;
        }

        // If sensors see different colors (not just ball vs no-ball),
        // it might indicate multiple balls
        return (verdict1 != 0 && verdict2 != 0 && verdict1 != verdict2);
    }

}