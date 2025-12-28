package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
    private Limelight3A limelight;
    double yawToTagBlue =0;
    double yawToTagRed =0;
    double targetTicks;
    boolean runningAround;
    boolean doneRUnning;
    double groundDistanceCM;
    double turretPoseRad;
    // Camera position relative to ROBOT CENTER (meters)
    public static final double CAM_X = 0.14; // forward (+X)
    public static final double CAM_Y = 0.00; // left (+Y)

    // Camera yaw relative to turret (rad)
    public static final double CAM_YAW_OFFSET = 0.0;
    public static final double METERS_TO_INCHES = 39.37007874;



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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
//        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.shareWasPressed()){
                TurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            pid.setPID(p, i, d);

            double currentPos = TurretMotor.getCurrentPosition();
            // Convert ticks → degrees → radians
            double turretDeg = currentPos / DegtoTickCoefficient;

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

            LLResult result = limelight.getLatestResult();

//            if (result != null && result.isValid()) {
//
//
//            }
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                Pose3D camToTag = fr.getCameraPoseTargetSpace();
                groundDistanceCM = Math.hypot(camToTag.getPosition().x, camToTag.getPosition().z)*100;
                if (fr.getFiducialId() == 20) {
                    yawToTagBlue = fr.getTargetXDegrees();
                    Pose3D llPose = fr.getRobotPoseTargetSpace();// FIELD pose of camera
                    telemetry.addData("LL Botpose X (m)", llPose.getPosition().x);
                    telemetry.addData("LL Botpose Y (m)", llPose.getPosition().y);
                    telemetry.addData("LL Botpose Heading",
                            llPose.getOrientation().getYaw());
                    Pose2d camFieldPose = new Pose2d(llPose.getPosition().x, llPose.getPosition().y, new Rotation2d(Math.toRadians(llPose.getOrientation().getYaw())));

                    Pose2d robotFieldPose = getRobotPoseFromLimelight(camFieldPose, camPoseRobot);
                    Pose2d robotFieldPoseInches = new Pose2d(
                            robotFieldPose.getX() * METERS_TO_INCHES,
                            robotFieldPose.getY() * METERS_TO_INCHES,
                            robotFieldPose.getRotation()
                    );

                    telemetry.addData("Robot X", robotFieldPoseInches.getX());
                    telemetry.addData("Robot Y", robotFieldPoseInches.getY());
                    telemetry.addData("Robot Heading deg",
                            Math.toDegrees(robotFieldPoseInches.getHeading()));
                    hasTag = true;
                }

                if (fr.getFiducialId() == 24) {
                    yawToTagRed = fr.getTargetXDegrees();
                    hasTag = true;
                }

            }



            if (mode == Mode.DRIVER) {
                // Manual control
                power = -gamepad1.left_stick_y;
            } else if(mode == Mode.ToTarget) {
                // PID control
                double error = target - currentPos;

                if (Math.abs(error) <= tolerance) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, target);
                    power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
                }

            }else if(mode == Mode.ToDegrees){
                // PID control
                double targetTicks = targetDegrees* DegtoTickCoefficient;
                double error = targetTicks - currentPos;

                if (Math.abs(error) <= tolerance) {
                    power = 0.0;
                } else {
                    double pidOutput = pid.calculate(currentPos, targetTicks);
                    power = Math.max(-turetSpeed, Math.min(turetSpeed, pidOutput));
                }

            }else{
                if(!runningAround) {
                    targetTicks = currentPos + (yawToTagBlue * DegtoTickCoefficient);
                }
                if(targetTicks>730){
                    targetTicks-=1300;
                    runningAround = true;
                    doneRUnning = false;
                }
                else if(targetTicks<-850){
                    targetTicks+=1300;
                    runningAround = true;
                    doneRUnning = false;
                }
                else {
                    if(doneRUnning) {
                        runningAround = false;
                    }
                }
                double error = targetTicks - currentPos;
                if(Math.abs(error)<=3 && runningAround){
                    doneRUnning = true;
                }
                if (Math.abs(error) <= tolerance ||(!hasTag && !runningAround)) {
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
            telemetry.addData("Target", target);
            telemetry.addData("Encoder Position", currentPos);
            telemetry.addData("Error", target - currentPos);
            telemetry.addData("Servo Power", power);
            telemetry.addData("degrees to blue", yawToTagBlue);
            telemetry.addData("degrees to red", yawToTagRed);
            telemetry.addData("distance to tag cm", groundDistanceCM);
            telemetry.addData("whatever is in the fiducial result", fiducialResults);
            telemetry.addData("P", p);
            telemetry.addData("I", i);
            telemetry.addData("D", d);
            telemetry.update();
        }


    }
}
