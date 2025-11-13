package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class launch_lift {

    //    public Servo liftServo;
    public Servo launchServo;
    public Servo scooperRight;
    public Servo scooperLeft;

    public launch_lift(HardwareMap hardwareMap){
//        liftServo = hardwareMap.get(Servo.class, "lift");
        launchServo = hardwareMap.get(Servo.class, "launch_angle");
        scooperRight = hardwareMap.get(Servo.class, "scooperright");
        scooperRight.setDirection(Servo.Direction.REVERSE);
        scooperLeft = hardwareMap.get(Servo.class, "scooperleft");
//        liftServo.setDirection(Servo.Direction.REVERSE);
    }
    public void liftBoth(double position){
        scooperLeft.setPosition(position);
        scooperRight.setPosition(position);

    }
    public void down(){
        liftBoth(0);
    }
    public void up(){
        scooperRight.setPosition(0.31);
        scooperLeft.setPosition(0.27);
    }

    public void close(){
        launchServo.setPosition(0.48);
    }
    public void veryclose(){launchServo.setPosition(0.45);}
    public void far(){
        launchServo.setPosition(0.4);
    }
}



