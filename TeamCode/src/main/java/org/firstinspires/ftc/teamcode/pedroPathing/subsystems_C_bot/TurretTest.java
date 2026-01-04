package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Turret pid test", group = "tuning")
@Config
public class TurretTest extends LinearOpMode {

    private DcMotorEx TurretMotor;   // Motor port used to read encoder

    private PIDController pid;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double p = 0.005;
    public static double i = 0.1;
    public static double d = 0.0001;
    public static int target = 0;
    public static double tolerance = 1.0;
    public static double turetSpeed = 0.8;
    public static double targetDegrees = 0.0;
    double DegtoTickCoefficient = 67.0/18.0;

    private enum Mode { DRIVER, ToTarget, ToDegrees,Limelight}
    private Mode mode = Mode.DRIVER;
//    private Limelight3A limelight;
    double lastTimestamp = -1;
    double yawToTag=0;
    boolean blueAlliance = false;
    double targetTicks;
    boolean runningAround;
    boolean doneRUnning;
    double groundDistanceCM;
    double turretPoseRad;
    private Follower follower;
    private final Pose startPose = new Pose(53,70,0); //red
    private final Pose blueGoal = new Pose(-72,144,0);
    private final Pose redGoal = new Pose(72,144,0);
    // Camera position relative to ROBOT CENTER (meters)
    public static final double CAM_X = 0.14; // forward (+X)
    public static final double CAM_Y = 0.00; // left (+Y)
    private static final double TURRET_MIN_TICKS = -850;
    private static final double TURRET_MAX_TICKS = 730;


    // Camera yaw relative to turret (rad)
    public static final double CAM_YAW_OFFSET = 0.0;
    public static final double METERS_TO_INCHES = 39.37007874;
    double headingTotag;
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 20, -90, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Pose ftcPose, pedroPose;
    boolean tagDetected;
    boolean tagInitializing;
    double turretDeg;

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            try { builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); }
            catch (Exception e) { telemetry.addLine("Warning: Webcam not found"); }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
    private void updateAprilTagLocalization() {
        if (aprilTag == null) return;

        List<AprilTagDetection> dets = aprilTag.getDetections();
        tagDetected = false;

        for (AprilTagDetection d : dets) {
            if (d.metadata == null) continue;
            if (d.metadata.name.contains("Obelisk")) continue;

            tagDetected = true;

            double xIn = d.robotPose.getPosition().x;
            double yIn = d.robotPose.getPosition().y;
            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            double bearing = d.ftcPose.z;
            double bearingDeg = 1.634*bearing;

            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            pedroPose = new Pose(ftcPose.getY(), -ftcPose.getX() + 72, ftcPose.getHeading()+Math.toRadians(turretDeg-bearingDeg));
            break;
        }
    }

    public static Pose2d getCameraPoseRobotFrame(double turretAngleRad) {

        // Rotate camera offset by turret angle
        double cos = Math.cos(turretAngleRad);
        double sin = Math.sin(turretAngleRad);

        double camX = CAM_X * cos - CAM_Y * sin;
        double camY = CAM_X * sin + CAM_Y * cos;

        double camHeading = turretAngleRad + CAM_YAW_OFFSET;


        return new Pose2d(camX, camY, new Rotation2d(camHeading));
    }
    public static Pose2d getRobotPoseFromLimelight(Pose2d camFieldPose, Pose2d camRobotPose) {
        // Robot heading = camera heading - camera relative heading
        double robotHeading = camFieldPose.getHeading() - camRobotPose.getHeading();

        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        // Rotate camera offset into FIELD frame
        double offsetX = camRobotPose.getX() * cos - camRobotPose.getY() * sin;
        double offsetY = camRobotPose.getX() * sin + camRobotPose.getY() * cos;

        double robotX = camFieldPose.getX() - offsetX;
        double robotY = camFieldPose.getY() - offsetY;

        return new Pose2d(robotX, robotY, new Rotation2d(robotHeading));
    }
    @Override
    public void runOpMode() {
        TurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(p, i, d);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        initAprilTag();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        /*
         * Starts polling for data.
         */
//        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cur = follower.getPose();
            double currentPos = TurretMotor.getCurrentPosition();
            // Convert ticks → degrees → radians
            turretDeg = currentPos / DegtoTickCoefficient;
            if(gamepad1.dpadUpWasPressed()){
                tagInitializing = !tagInitializing;
            }
            if (tagInitializing) {
                updateAprilTagLocalization();
                if (tagDetected && pedroPose != null && !follower.isBusy()) {
                    telemetry.addLine("seeing and localizing tag");
                    telemetry.addData("local x",pedroPose.getX());
                    telemetry.addData("local y",pedroPose.getY());
                    telemetry.addData("local hed",Math.toDegrees(pedroPose.getHeading()));
//                    follower.setPose(pedroPose.getPose());
//                    tagInitializing = false;
                }
            }

            if(gamepad1.psWasPressed()){
                blueAlliance = !blueAlliance;
                Pose invert = new Pose(-cur.getX(),cur.getY(),cur.getHeading()+Math.toRadians(180));
                follower.setPose(invert);
            }
            if (gamepad1.shareWasPressed()) {
                TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            Pose targett = blueAlliance ? blueGoal : redGoal;
            double rawAngle = Math.atan2(targett.getY() - cur.getY(), targett.getX() - cur.getX());

            double flippedAngle = rawAngle + Math.PI;
            flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

            headingTotag = flippedAngle+Math.PI;
            pid.setPID(p, i, d);



            // Invert so CCW is positive
            turretPoseRad = Math.toRadians(-turretDeg);

            Pose2d camPoseRobot = getCameraPoseRobotFrame(turretPoseRad);

            telemetry.addData("Turret deg", Math.toDegrees(turretPoseRad));

            telemetry.addData("Cam X (m)", camPoseRobot.getX());
            telemetry.addData("Cam Y (m)", camPoseRobot.getY());
            telemetry.addData("Cam Heading deg", Math.toDegrees(camPoseRobot.getHeading()));

            if (gamepad1.triangleWasPressed()) mode = Mode.ToTarget;
            if (gamepad1.circleWasPressed()) mode = Mode.DRIVER;
            if (gamepad1.squareWasPressed()) mode = Mode.ToDegrees;
            if (gamepad1.crossWasPressed()) mode = Mode.Limelight;

            double power;
            boolean hasTag = false;

//            LLResult result = limelight.getLatestResult();

//            if (result != null && result.isValid()) {
//
//
//            }
//            List<LLResultTypes.FiducialResult> fiducialResults = null;
//            if (result.getTimestamp() != lastTimestamp) {
//                lastTimestamp = result.getTimestamp();
//                fiducialResults = result.getFiducialResults();
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    if(blueAlliance){
//                        if (fr.getFiducialId() == 20) {
//                            yawToTag = fr.getTargetXDegrees();
//                            hasTag = true;
//                            Pose3D camToTag = fr.getCameraPoseTargetSpace();
//                            groundDistanceCM = Math.hypot(camToTag.getPosition().x, camToTag.getPosition().z)*100;
//                        }
//                        else{
//                            yawToTag = 0;
//                            hasTag = false;
//                        }
//                    }
//                    else {
//                        if (fr.getFiducialId() == 24) {
//                            yawToTag = fr.getTargetXDegrees();
//                            hasTag = true;
//                            Pose3D camToTag = fr.getCameraPoseTargetSpace();
//                            groundDistanceCM = Math.hypot(camToTag.getPosition().x, camToTag.getPosition().z)*100;
//                        }
//                        else{
//                            yawToTag = 0;
//                            hasTag = false;
//                        }
//                    }
//
//                }
//            }


            if (mode == Mode.DRIVER) {
                // Manual control
                power = -gamepad1.left_stick_y;
            } else if (mode == Mode.ToTarget) {
                // PID control
                double error = target - currentPos;

                if (Math.abs(error) <= tolerance) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, target);
                    power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
                }

            } else if (mode == Mode.ToDegrees) {

                double targetDegrees2 = Math.toDegrees(cur.getHeading() - headingTotag);
                double targetTicks = targetDegrees2 * DegtoTickCoefficient;

                // Clamp target to limits
                targetTicks = Math.max(TURRET_MIN_TICKS,
                        Math.min(TURRET_MAX_TICKS, targetTicks));

                double error = targetTicks - currentPos;

                if (Math.abs(error) <= tolerance) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, targetTicks);
                    power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
                }
            } else {
                if (!runningAround) {
                    targetTicks = currentPos + (yawToTag * DegtoTickCoefficient);
                }
                if (targetTicks > 730) {
                    targetTicks -= 1300;
                    runningAround = true;
                    doneRUnning = false;
                } else if (targetTicks < -850) {
                    targetTicks += 1300;
                    runningAround = true;
                    doneRUnning = false;
                } else {
                    if (doneRUnning) {
                        runningAround = false;
                    }
                }
                double error = targetTicks - currentPos;
                if (Math.abs(error) <= 3 && runningAround) {
                    doneRUnning = true;
                }
                if (Math.abs(error) <= tolerance || (!hasTag && !runningAround)) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, targetTicks);
                    power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
                }
            }

            TurretMotor.setPower(power);
            //LLResult result = limelight.getLatestResult();
//            if (result != null) {
//                if (result.isValid()) {
//                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("ty", result.getTy());
////                    telemetry.addData("Botpose", botpose.toString());
//                    telemetry.addData("robot yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
//                }
//            }


            telemetry.addData("Mode", mode);
            telemetry.addData("follower x",cur.getX());
            telemetry.addData("follower y",cur.getY());
            telemetry.addData("follower heading",Math.toDegrees(cur.getHeading()));
            telemetry.addData("desired angle to tag",Math.toDegrees(headingTotag));
            telemetry.addData("Target", target);
            telemetry.addData("Encoder Position", currentPos);
            telemetry.addData("Error", target - currentPos);
            telemetry.addData("Servo Power", power);
            telemetry.addData(blueAlliance?"degrees to blue tag":"degrees to red tag", yawToTag);
            telemetry.addData("distance to tag cm", groundDistanceCM);
//            telemetry.addData("whatever is in the fiducial result", fiducialResults);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.update();
        }


    }
}
