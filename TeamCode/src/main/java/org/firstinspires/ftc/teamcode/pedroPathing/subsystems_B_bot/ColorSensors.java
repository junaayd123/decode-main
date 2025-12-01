package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensors {

    public NormalizedColorSensor SensorLeft;
    public NormalizedColorSensor SensorRight;
    public NormalizedColorSensor SensorBack;
    public NormalizedColorSensor SensorLeft2;
    public NormalizedColorSensor SensorRight2;
    public NormalizedColorSensor SensorBack2;
    public ColorSensors(HardwareMap hardwareMap) {
        SensorBack = hardwareMap.get(NormalizedColorSensor.class, "color_back");
        SensorLeft = hardwareMap.get(NormalizedColorSensor.class, "color_left");
        SensorRight = hardwareMap.get(NormalizedColorSensor.class, "color_right");
        SensorBack2 = hardwareMap.get(NormalizedColorSensor.class, "color_back2");
        SensorLeft2 = hardwareMap.get(NormalizedColorSensor.class, "color_left2");
        SensorRight2 = hardwareMap.get(NormalizedColorSensor.class, "color_right2");
    }
    public int getColor(NormalizedColorSensor colorSensor){
        // 1 is green, 2 is purple

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;



        // --- Step 1: Detect if no ball is present ---
        if (red < 0.005 && green < 0.005 && blue < 0.005) {
            return 0;//0 is no ball
        }
        // --- Step 2: Detect Green vs Purple ---
        else if (green > blue && green > red) {
            return 1; //1 is green ball
        } else if (blue >= green && blue > red) {
            return 2; //2 is purp ball
        }
        else return 0;
    }

    public int getRight(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorRight);
        verdict2 = getColor(SensorRight2);
        if(verdict1==1||verdict2==1){
            return 1;
        }
        else if(verdict1==2||verdict2==2){
            return 2;
        }
        else return 0;
    }

    public int getLeft(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorLeft);
        verdict2 = getColor(SensorLeft2);
        if(verdict1==1||verdict2==1){
            return 1;
        }
        else if(verdict1==2||verdict2==2){
            return 2;
        }
        else return 0;
    }
    public int getBack(){
        int verdict1, verdict2;
        verdict1 = getColor(SensorBack);
        verdict2 = getColor(SensorBack2);
        if(verdict1==1||verdict2==1){
            return 1;
        }
        else if(verdict1==2||verdict2==2){
            return 2;
        }
        else return 0;
    }

}
