package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class lift_three {
    public Servo launchAngleServo;
    public Servo liftRight;
    public Servo liftLeft;
    public Servo liftBack;
    public lift_three(HardwareMap hardwareMap){
        launchAngleServo = hardwareMap.get(Servo.class, "launch_angle");
        liftRight = hardwareMap.get(Servo.class, "lift_right");
        liftBack = hardwareMap.get(Servo.class, "lift_back");
        liftLeft = hardwareMap.get(Servo.class, "lift_left");
        liftLeft.setDirection(Servo.Direction.REVERSE);
        liftBack.setDirection(Servo.Direction.REVERSE);
    }
    public void leftDown(){liftLeft.setPosition(0);}
    public void leftUp(){liftLeft.setPosition(0.28);}
    public void rightDown(){liftRight.setPosition(0);}
    public void rightUp(){liftRight.setPosition(0.26);}
    public void backDown(){liftBack.setPosition(0);}
    public void backUp(){liftBack.setPosition(0.28);}
    public void allDown(){
        liftBack.setPosition(0);
        liftLeft.setPosition(0);
        liftRight.setPosition(0);
    }


}
