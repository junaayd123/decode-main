// Modified JunaaydCombinedTeleOp
// Triangle HELD -> continuous AprilTag localization
// Square PRESSED -> auto path to red/blue shooting pose, then returns to teleop

package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.launch_lift;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.A_Bot_Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Junaayd Combined TeleOp Modified", group = "TeleOp")
public class aprilTagTeleopMisha extends OpMode {

    private boolean aligning = false;
    private DcMotor intake = null;
    private Deposition depo;
    private boolean bluealliance = false;

    Gamepad preG1 = new Gamepad();
    Timer timer1, timer2, timer3, timer4, timer5;

    Gamepad g1 = new Gamepad();
    Gamepad preG2 = new Gamepad();
    Gamepad g2 = new Gamepad();
    private launch_lift LL;
    private double speed = 1;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0.0);
    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(135.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(45.0));

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public boolean tagDetected;

    Pose pedroPose, ftcPose;

    @Override
    public void init() {
        LL = new launch_lift(hardwareMap);
        follower = A_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = hardwareMap.get(DcMotor.class, "intake");
        depo = new Deposition(hardwareMap);

        timer1 = new Timer();
        timer2 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer5 = new Timer();

        g1.copy(gamepad1);
        g2.copy(gamepad2);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
        LL.down();
        LL.close();
        timer1.resetTimer(); timer2.resetTimer(); timer3.resetTimer(); timer4.resetTimer(); timer5.resetTimer();
        depo.setTargetVelocity(0);
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        depo.updatePID();

        if (g1.cross) speed = 0.3; else speed = 1;

        handleIntakeAndDepo();

        // ========= APRILTAG LOCALIZATION WHILE TRIANGLE IS HELD =========
        if (g1.triangle) {
            updateAprilTagLocalization();
            if (tagDetected && pedroPose != null && !follower.isBusy()) {
                follower.setPose(pedroPose.getPose());
            }
        }

        // ========= WHEN SQUARE PRESSED → DRIVE TO SHOOTING POSE =========
        if (g1.square && !preG1.square) {
            if (!follower.isBusy()) {
                goToAllianceShootingPose();
            }
        }

        followerstuff();

        telemetry.addData("Alliance Blue?", bluealliance);
        telemetry.update();
    }

    // ---------- AprilTag INIT ----------
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

    // ---------- APRILTAG UPDATE (for triangle-held localization) ----------
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

            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));

            pedroPose = new Pose(ftcPose.getY() + 72, -ftcPose.getX() + 72, ftcPose.getHeading());
            break;
        }
    }

    // ---------- DRIVE TO RED OR BLUE SHOOTING POSE ----------
    private void goToAllianceShootingPose() {
        Pose target = bluealliance ? blueNearShootPose : redNearShootPose;

        if (pedroPose == null) return;

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(pedroPose, target))
                .setLinearHeadingInterpolation(pedroPose.getHeading(), target.getHeading())
                .build();

        follower.followPath(chain);
    }

    // ---------- intake + deposition ----------
    private void handleIntakeAndDepo() {
        boolean allTimersOff = !timer1.timerIsOn() && !timer2.timerIsOn() && !timer3.timerIsOn() && !timer4.timerIsOn() && !timer5.timerIsOn();

        if (g2.right_bumper && allTimersOff) intake.setPower(-1.0);
        else if (g2.left_bumper && !preG2.left_bumper && allTimersOff) timer5.startTimer();
        else if (allTimersOff) intake.setPower(0);
    }

    private void followerstuff(){
        follower.update();

        if (!follower.isBusy()) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speed,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                    -gamepad1.right_stick_x * speed,
                    true
            );
        }
    }
}
