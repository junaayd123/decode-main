package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTestTeleOP extends LinearOpMode {
    public float normalizedRadians;
    public double xOffset;
    public double yOffset;
    public double xCenter;
    public double yCenter;
    public Follower follower;
    public PathBuilder path;
    public Pose currentRobotPose;

    @Override
    public void runOpMode() throws InterruptedException{
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640,480)).build();
        waitForStart();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        while (!isStopRequested()&&opModeIsActive()){
            if(!tagProcessor.getDetections().isEmpty()){
                for(int i=0; i<tagProcessor.getDetections().size(); i++) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(i);
                    try {
                        //normalizedRadians = (float)Math.toRadians(tag.ftcPose.bearing-tag.ftcPose.yaw);
                        //yOffset = Math.sin((double)normalizedRadians)*tag.ftcPose.range;
                        //xOffset = Math.cos((double)normalizedRadians)*tag.ftcPose.range;
                        telemetry.addData(i + " x", tag.ftcPose.x);
                        telemetry.addData(i + " y", tag.ftcPose.y);
                        telemetry.addData(i + " z", tag.ftcPose.z);
                        telemetry.addData(i + " roll", tag.ftcPose.roll);
                        telemetry.addData(i + " pitch", tag.ftcPose.pitch);
                        telemetry.addData(i + " yaw", tag.ftcPose.yaw);
                    } catch (Exception e) {
                        telemetry.addData("exception: ", e);
                        telemetry.addData(i + " x", "NULL");
                        telemetry.addData(i + " y", "NULL");
                        telemetry.addData(i + " z", "NULL");
                        telemetry.addData(i + " roll", "NULL");
                        telemetry.addData(i + " pitch", "NULL");
                        telemetry.addData(i + " yaw", "NULL");

                    }
                }
                if (gamepad1.a && !follower.isBusy() && !tagProcessor.getDetections().isEmpty()) {
                    currentRobotPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
                    follower.setStartingPose(currentRobotPose);
                    try {} catch {}
                    double tempX = tagProcessor.getDetections().get(1).ftcPose.x;
                    double tempX = tagProcessor.getDetections().get(1).ftcPose.x;
                    PathBuilder path = follower.pathBuilder()
                            .setLinearHeadingInterpolation(0, 90)
                            .addPath(new BezierLine(currentRobotPose, ));
                    follower.followPath(path);
                }
            }
            telemetry.update();
        }
    }
}