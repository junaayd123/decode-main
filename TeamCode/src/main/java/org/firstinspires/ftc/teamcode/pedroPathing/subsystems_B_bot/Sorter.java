package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sorter {
    private CRServo servo;
    private DcMotorEx encoderMotor;   // Motor port used to read encoder


    private PIDController pid;
    public static double p = 0.00023;
    public static double i = 0.07;
    public static double d = 0.000025;
    public Sorter(HardwareMap hardwareMap){
        AnalogInput absoluteEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        servo = hardwareMap.get(CRServo.class, "sorterservo");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "tbenc");
        pid = new PIDController(p, i, d);
    }

}
