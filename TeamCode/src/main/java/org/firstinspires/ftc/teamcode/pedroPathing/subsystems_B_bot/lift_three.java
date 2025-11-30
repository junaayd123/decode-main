package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lift_three {
    public ColorSensors sensors;
    public Servo launchAngleServo;
    public Servo liftRight;
    public Servo liftLeft;
    public Servo liftBack;
    public lift_three(HardwareMap hardwareMap){
        sensors = new ColorSensors(hardwareMap);
        launchAngleServo = hardwareMap.get(Servo.class, "launch_angle");
        liftRight = hardwareMap.get(Servo.class, "lift_right");
        liftBack = hardwareMap.get(Servo.class, "lift_back");
        liftLeft = hardwareMap.get(Servo.class, "lift_left");
        liftLeft.setDirection(Servo.Direction.REVERSE);
        liftBack.setDirection(Servo.Direction.REVERSE);
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
    public boolean checkNoBalls(){//false if theres at least one ball true if no balls
        if(sensors.getBack()+sensors.getLeft()+sensors.getRight()==0) return true;
        else return false;
    }
    public void leftDown(){liftLeft.setPosition(0);}
    public void leftUp(){liftLeft.setPosition(0.28);}
    public void rightDown(){liftRight.setPosition(0);}
    public void rightUp(){liftRight.setPosition(0.26);}
    public void backDown(){liftBack.setPosition(0);}
    public void backUp(){liftBack.setPosition(0.28);}
    public void set_angle_min(){launchAngleServo.setPosition(0.06);}
    public void set_angle_far(){launchAngleServo.setPosition(0.18);}
    public void set_angle_far_auto(){launchAngleServo.setPosition(0.16);}
    public void set_angle_close(){launchAngleServo.setPosition(0.06);}

    public void allDown(){
        liftBack.setPosition(0);
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
    }


}
