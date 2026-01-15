package org.firstinspires.ftc.teamcode.pedroPathing.Misc;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.jetbrains.annotations.TestOnly;

@TeleOp(name = "apriltaglime")
public class AprilTagLimeLight extends OpMode {
    private Limelight3A limelight;
    private IMU imu;

    private double distance;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot rebHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(rebHubOrientationOnRobot));



    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()){
            Pose3D botpose = llresult.getBotpose();
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Ta", llresult.getTa());
            telemetry.addData("botPose", botpose.toString());

        }

    }
}
