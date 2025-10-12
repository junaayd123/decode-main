package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Deposition {
    public DcMotor left;
    public DcMotor right;
    public double closePower = 0.67;
    public double farPower = 0.85;
    public Deposition(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotor.class, "depo");
        right = hardwareMap.get(DcMotor.class, "depo1");
    }
    public  void setPowerBoth(double power){
        left.setPower(power);
        right.setPower(power);
    }
    public void shootClose(){
        setPowerBoth(closePower);
    }
    public void shootFar(){
        setPowerBoth(farPower);
    }
}