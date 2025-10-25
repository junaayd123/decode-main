package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class tbenctesting extends LinearOpMode{
    DcMotor tbenc;
    public void runOpMode() throws InterruptedException{
        double rotations;
        tbenc = hardwareMap.get(DcMotor.class, "tbenc");
        waitForStart();
        while (!isStopRequested()&&opModeIsActive()) {
            rotations = ((double)tbenc.getCurrentPosition())/8192.0);
            telemetry.addData("Encoder Data", tbenc.getCurrentPosition());
            telemetry.addData("Full rotation:", ((double)tbenc.getCurrentPosition())/8192.0);

            telemetry.update();
        }
    }
}
