package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class launch_lift {

    public Servo liftServo;
    public Servo launchServo;

    public launch_lift(HardwareMap hardwareMap){
        liftServo = hardwareMap.get(Servo.class, "lift");
        launchServo = hardwareMap.get(Servo.class, "launch_angle");
        liftServo.setDirection(Servo.Direction.REVERSE);
    }

    public void down(){liftServo.setPosition(0);}
    public void up(){liftServo.setPosition(1);}

    public void close(){
        launchServo.setPosition(0.4);
    }
    public void veryclose(){launchServo.setPosition(0.45);}
    public void far(){
        launchServo.setPosition(0.4);
    }



}