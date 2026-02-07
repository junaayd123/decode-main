package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lift_three {
    public ColorSensors sensors;
    public Servo launchAngleServo;
    public Servo liftRight;
    public Servo liftLeft;
    public Servo liftBack;

    // Track previous sensor states to detect when balls leave chambers
    private int prevLeftState = 0;
    private int prevRightState = 0;
    private int prevBackState = 0;

    // Track actual ball count
    private int actualBallCount = 0;

    public lift_three(HardwareMap hardwareMap){
        sensors = new ColorSensors(hardwareMap);
        launchAngleServo = hardwareMap.get(Servo.class, "launch_angle");
        liftRight = hardwareMap.get(Servo.class, "lift_right");
        liftBack = hardwareMap.get(Servo.class, "lift_back");
        liftLeft = hardwareMap.get(Servo.class, "lift_left");
        liftLeft.setDirection(Servo.Direction.REVERSE);
        liftBack.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Call this method in your loop() to continuously update ball tracking
     * This detects when balls move between chambers or leave the robot
     */
    public void updateBallTracking() {
        int currentLeft = sensors.getLeft();
        int currentRight = sensors.getRight();
        int currentBack = sensors.getBack();

        // Detect balls entering chambers (0 -> non-zero)
        if(prevLeftState == 0 && currentLeft != 0) {
            actualBallCount++;
        }
        if(prevRightState == 0 && currentRight != 0) {
            actualBallCount++;
        }
        if(prevBackState == 0 && currentBack != 0) {
            actualBallCount++;
        }

        // Detect balls leaving chambers (non-zero -> 0)
        if(prevLeftState != 0 && currentLeft == 0) {
            actualBallCount--;
        }
        if(prevRightState != 0 && currentRight == 0) {
            actualBallCount--;
        }
        if(prevBackState != 0 && currentBack == 0) {
            actualBallCount--;
        }

        // Clamp ball count to valid range [0, 3]
        if(actualBallCount < 0) actualBallCount = 0;
        if(actualBallCount > 3) actualBallCount = 3;

        // Update previous states
        prevLeftState = currentLeft;
        prevRightState = currentRight;
        prevBackState = currentBack;
    }

    /**
     * Get the actual number of balls in the robot
     * @return number of balls (0-3)
     */
    public int getBallCount() {
        return actualBallCount;
    }

    /**
     * Get current chamber occupancy
     * @return number of chambers with balls detected
     */
    public int getOccupiedChambers() {
        int count = 0;
        if(sensors.getLeft() != 0) count++;
        if(sensors.getRight() != 0) count++;
        if(sensors.getBack() != 0) count++;
        return count;
    }

    /**
     * Check if all chambers are full (3 balls in position)
     * @return true if all three sensors detect balls
     */
    public boolean allChambersFull() {
        return sensors.getLeft() != 0 && sensors.getRight() != 0 && sensors.getBack() != 0;
    }

    /**
     * Reset ball count (use when you know the robot is empty, like at start)
     */
    public void resetBallCount() {
        actualBallCount = 0;
        prevLeftState = sensors.getLeft();
        prevRightState = sensors.getRight();
        prevBackState = sensors.getBack();
    }

    public boolean lift_green(){
        if(sensors.getRight()==1){
            rightUp();
            return true;
        }
        else if(sensors.getLeft()==1){
            leftUp();
            return true;
        }
        else if(sensors.getBack()==1){
            backUp();
            return true;
        }
        else return false;
    }

    public boolean lift_purple(){
        if(sensors.getRight()==2){
            rightUp();
            return true;
        }
        else if(sensors.getLeft()==2){
            leftUp();
            return true;
        }
        else if(sensors.getBack()==2){
            backUp();
            return true;
        }
        else return false;
    }

    public int lift_green2(boolean testRight,boolean testLeft, boolean testBack){
        if(testRight) {
            if (sensors.getRight() == 1) {
                rightUp();
                return 1;
            }
        }
        if(testLeft) {
            if (sensors.getLeft() == 1) {
                leftUp();
                return 0;
            }
        }
        if(testBack) {
            if (sensors.getBack() == 1) {
                backUp();
                return 2;
            }
        }
        return -1;
    }

    public int lift_purple2(boolean testRight,boolean testLeft, boolean testBack){
        if(testRight) {
            if (sensors.getRight() == 2) {
                rightUp();
                return 1;
            }
        }
        if(testLeft) {
            if (sensors.getLeft() == 2) {
                leftUp();
                return 0;
            }
        }
        if(testBack) {
            if (sensors.getBack() == 2) {
                backUp();
                return 2;
            }
        }
        return -1;
    }

    public boolean checkNoBalls(){//false if theres at least one ball true if no balls
        if(sensors.getBack()+sensors.getLeft()+sensors.getRight()==0) return true;
        else return false;
    }

    public void leftDown(){liftLeft.setPosition(0);}
    public void leftUp(){liftLeft.setPosition(0.3);}
    public void rightDown(){liftRight.setPosition(0);}
    public void rightUp(){liftRight.setPosition(0.3);}
    public void backDown(){liftBack.setPosition(0);}
    public void backUp(){liftBack.setPosition(0.31);}
    public void set_angle_min(){launchAngleServo.setPosition(0.06);}
    public void set_angle_custom(double ang){launchAngleServo.setPosition(ang);}
    public void set_angle_far(){launchAngleServo.setPosition(0.18);}
    public void set_angle_far_auto(){launchAngleServo.setPosition(0.20);}
    public void set_angle_close(){launchAngleServo.setPosition(0.17);}

    public void allDown(){
        liftBack.setPosition(0);
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
    }
}
