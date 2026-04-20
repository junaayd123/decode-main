package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lifters {
    public ColorSensors_New sensors;
    public Servo launchAngleServo;
    public Servo liftRight;
    public Servo liftLeft;
    public Servo liftBack;
    public Servo camera;
    //
    public lifters(HardwareMap hardwareMap){
        sensors = new ColorSensors_New(hardwareMap);
        launchAngleServo = hardwareMap.get(Servo.class, "launch_angle");
        liftRight = hardwareMap.get(Servo.class, "lift_right");
        liftBack = hardwareMap.get(Servo.class, "lift_back");
        liftLeft = hardwareMap.get(Servo.class, "lift_left");
        camera = hardwareMap.get(Servo.class, "cam_tilt");
//        liftLeft.setDirection(Servo.Direction.REVERSE);
//        liftBack.setDirection(Servo.Direction.REVERSE);
        liftRight.setDirection(Servo.Direction.REVERSE);
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

    public void leftDown(){liftLeft.setPosition(0.07);}
    public void leftUp(){liftLeft.setPosition(0.39);}
    public void rightDown(){liftRight.setPosition(0.01);}
    public void rightUp(){liftRight.setPosition(0.33);}
    public void backDown(){liftBack.setPosition(0.01);}
    public void backUp(){liftBack.setPosition(0.35);}

    public void set_angle_min(){launchAngleServo.setPosition(0.08);}
    public void set_angle_custom(double ang){launchAngleServo.setPosition(ang);}
    //public void set_angle_far(){launchAngleServo.setPosition(0.18);}
    public void set_angle_far(){launchAngleServo.setPosition(0.21);}
    public void set_angle_far_firstshot(){launchAngleServo.setPosition(0.21);}
    public void set_angle_far_auto(){launchAngleServo.setPosition(0.19);}
    public double farShotPos = 0.22;
    public void set_angle_far_auto2(){launchAngleServo.setPosition(farShotPos);}
    public void set_angle_farredoptimized(){launchAngleServo.setPosition(0.23);}
    public void set_angle_farblueoptimized(){launchAngleServo.setPosition(0.23);}

    public void set_angle_close(){launchAngleServo.setPosition(0.2);}
    public void set_angle_mid(){launchAngleServo.setPosition(0.1);}
    public void set_angle_close2(){launchAngleServo.setPosition(0.11);}
    public void set_camera_tag_pos(){camera.setPosition(0.36);}
    public void set_camera_ramp_pos(){camera.setPosition(0.24);}
    public void set_camera_obelisk_pos(){camera.setPosition(0.24);}

    public void allDown(){
        leftDown();
        rightDown();
        backDown();
    }


}