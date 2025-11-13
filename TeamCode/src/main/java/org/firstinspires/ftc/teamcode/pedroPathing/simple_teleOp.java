package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "SimpleTeleOp", group = "TeleOp")
public class simple_teleOp extends OpMode {

    private DcMotor intake = null;
    private Deposition depo;
    Gamepad preG1= new Gamepad();;
    public double curTime = 10000;
    private ElapsedTime timer = new ElapsedTime();
    Gamepad g1= new Gamepad();;
    Gamepad preG2= new Gamepad();;
    Gamepad g2= new Gamepad();;
    private launch_lift LL;
    private boolean isFar = false;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640,480)).build();
        LL = new launch_lift(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = hardwareMap.get(DcMotor.class, "intake");
        depo = new Deposition(hardwareMap);
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        // Optional: set directions if your motors are mounted opposite each other
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // depo.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        LL.down();
        LL.close();
        timer.reset();
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        //intake with gampad cross
        if (gamepad1.a) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
        if(g1.right_bumper && !preG1.right_bumper){
            LL.up();
        }
        else if(g1.left_bumper && !preG1.left_bumper){
            LL.down();
        }
        if (g1.dpad_right &&!preG1.dpad_right) {
            LL.launchServo.setPosition(LL.launchServo.getPosition()+0.1);
        }
        else if (g1.dpad_left &&!preG1.dpad_left) {
            LL.launchServo.setPosition(LL.launchServo.getPosition()-0.1);
        }

        // Depo with gamepad cirlce
        if (gamepad1.b && !preG1.b) {
            curTime = timer.seconds();
            depo.shootClose();
            LL.close();
            isFar = false;
        }
        if (g1.square && !preG1.square){
            curTime = timer.seconds();
            depo.shootFar();
            LL.far();
            isFar = true;
        }
        if(isFar){
            farshoot();
        }
        else{
            closeshoot();
        }


        follower.setTeleOpDrive( -gamepad1.left_stick_y, (gamepad1.left_trigger - gamepad1.right_trigger), -gamepad1.right_stick_x, true );
        follower.update();

        // Telemetry
        telemetry.addData("Launch Position", LL.launchServo.getPosition());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Depo Power", depo.right.getPower());
        telemetry.addData("time since run", timer.seconds()-curTime);
        telemetry.update();
    }
    public void farshoot(){
        if(timer.seconds() >= curTime && timer.seconds()<curTime+0.1){
            depo.shootFar();
            LL.far();
        }
        if(timer.seconds() >= curTime +.4 && timer.seconds()< curTime+0.5){
            LL.up();
        }
        if (timer.seconds()>=curTime+1.1 && timer.seconds() < curTime+1.2){

            LL.down();

        }
        if(timer.seconds()>=curTime+1.7 && timer.seconds() < curTime+1.8){
            depo.setPowerBoth(0.0);
            curTime = 10000;
        }
    }

    public void closeshoot(){
        if(timer.seconds() >= curTime && timer.seconds()<curTime+0.1){
            depo.shootClose();
            LL.close();
        }
        if(timer.seconds() >= curTime +.8 && timer.seconds()< curTime+0.9){
            LL.up();
        }
        if (timer.seconds()>=curTime+1.5 && timer.seconds() < curTime+1.6){

            LL.down();

        }
        if(timer.seconds()>=curTime+2.1 && timer.seconds() < curTime+2.2){
            depo.setPowerBoth(0.0);
            curTime = 10000;
        }
    }
}