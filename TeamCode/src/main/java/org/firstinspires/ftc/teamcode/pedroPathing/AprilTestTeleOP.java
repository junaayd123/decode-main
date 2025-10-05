package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.TagCoordinate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class AprilTestTeleOP extends LinearOpMode {

    public TagCoordinate Tag20 = new TagCoordinate(5, 125, (float)29, -54);
    public TagCoordinate Tag24 = new TagCoordinate(5, 125, (float)29, 54); //FIX THIS
    public TagCoordinate blueShootLocation = new TagCoordinate(72, 20, 0, 0);

    public float normalizedRadians;
    public double xOffset;
    public double yOffset;
    public double xCenter;
    public double yCenter;
    public Follower follower;
    public PathBuilder path;
    public Pose startingPose;
    public Pose currentRobotPose;
    public double fieldCentricRotation;

    @Override
    public void runOpMode() throws InterruptedException{
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640,480)).build();
        waitForStart();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(83, 60, 0));

        while (!isStopRequested()&&opModeIsActive()){
            if (follower.isBusy()) {
                telemetry.addLine("Pedro busy");
            }
            if (gamepad1.a) {
                telemetry.addLine("A pressed");
            }
            follower.update();
            currentRobotPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
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
                        telemetry.addData(i + " range", tag.ftcPose.range);
                        telemetry.addData(i + " bearing", tag.ftcPose.bearing);
                        if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                            telemetry.addData("OBELISK DETECTED", (tag.id == 21) ? "G P P" : (tag.id == 22) ? "P G P" : "P P G");
                        }
                        if (tag.id == 20) {
                            fieldCentricRotation = tag.ftcPose.yaw + Tag20.getTagYaw();
                            //telemetry.addData("X to move to desired location", subtractCoordinateWithoutZ(new TagCoordinate(), Tag20));
                            telemetry.addData("Field centric Rotation", fieldCentricRotation);
                        }

                        double currentX = tag.ftcPose.range * Math.sin(Math.toRadians(tag.ftcPose.bearing - tag.ftcPose.yaw));
                        double currentY= tag.ftcPose.range * Math.cos(Math.toRadians(tag.ftcPose.bearing - tag.ftcPose.yaw));
                        telemetry.addData("Correct X", currentX);
                        telemetry.addData("Correct Y", currentY);
                        telemetry.addData("Relative Rotation", tag.ftcPose.bearing-tag.ftcPose.yaw);
                        telemetry.addData("ID number", tag.id);
                        //telemetry.addData("Fieldwise Rotation");

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
                telemetry.addData("Pedro X", currentRobotPose.getX());
                telemetry.addData("Pedro Y", currentRobotPose.getY());
                telemetry.addData("Pedro Heading", Math.toDegrees(currentRobotPose.getHeading()));
                if (gamepad1.b && !follower.isBusy() && !tagProcessor.getDetections().isEmpty()) {
                    currentRobotPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), fieldCentricRotation);
                    follower.setStartingPose(currentRobotPose);
                }
                if (gamepad1.a && !follower.isBusy() && !tagProcessor.getDetections().isEmpty()) {
                    currentRobotPose = new Pose(currentRobotPose.getX(), currentRobotPose.getY(), Math.toRadians(fieldCentricRotation));
                    follower.setStartingPose(currentRobotPose);
                    try {
                        AprilTagDetection detectionArray = tagProcessor.getDetections().get(0);
                        PathChain pathChain = follower.pathBuilder()
                                .addPath(new BezierLine(currentRobotPose, new Pose (currentRobotPose.getX()+1, currentRobotPose.getY()+1, 0)))
                                .setLinearHeadingInterpolation(Math.toRadians(fieldCentricRotation), 0)
                                .build();
                        follower.followPath(pathChain);
                    } catch (Exception e) {
                        telemetry.addData("Error...", e);
                    }

                }
            }
            telemetry.update();
        }
    }

    public TagCoordinate subtractCoordinateWithoutZ(TagCoordinate currentPosition, TagCoordinate tagPosition) {
        // do stuff
        TagCoordinate test = new TagCoordinate(tagPosition.getTagX() - currentPosition.getTagX(), tagPosition.getTagY() - currentPosition.getTagY(), tagPosition.getTagZ(), tagPosition.getTagYaw()-currentPosition.getTagYaw());
        return test;
    }
}