// Modified JunaaydCombinedTeleOp
// Triangle HELD -> continuous AprilTag localization
// Square PRESSED -> auto path to red/blue shooting pose, then returns to teleop

package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp with tags B", group = "TeleOp")
public class tagsteleopBotB extends OpMode {

    private boolean aligning = false;
    private boolean aligning2 = false;
    private double distanceToGoal;
    // Coordinates of red/blue speaker tags (meters)

    private DcMotor intake = null;
    private Deposition depo;
    private boolean bluealliance = false;
    private double desiredHeading = 0;

    Gamepad preG1 = new Gamepad();
    Timer timer1, timer2, timer3, timer4, timer5;

    Gamepad g1 = new Gamepad();
    Gamepad preG2 = new Gamepad();
    Gamepad g2 = new Gamepad();
    private launch_lift LL;
    private double speed = 1;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0.0);
    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(320.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(220.0));
    private final Pose redGoal  = new Pose(144, 144, 0);
    private final Pose blueGoal  = new Pose(0, 144,0);
    private final Pose redHP  = new Pose(42, 25, Math.toRadians(180)); //red human player
    private final Pose blueHP  = new Pose(115, 25,0); // blue human player

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public boolean tagDetected;

    Pose pedroPose, ftcPose;

    @Override
    public void init() {
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        timer1 = new Timer();
        timer2 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer5 = new Timer();

        g1.copy(gamepad1);
        g2.copy(gamepad2);


        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        if (g1.cross) speed = 0.3; else speed = 1;
        if(g1.psWasPressed()) bluealliance = !bluealliance;

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
                goToHumanPlayer();
            }
        }
        if (g1.circle && !preG1.circle && !follower.isBusy()) {
            faceAllianceGoal();
        }

        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();      // stops all paths and turns
            follower.startTeleopDrive();    // force drive mode back
            aligning = false;              // clear flags
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }
        // SMALL TURN UNSTICKING ONLY FOR FACE-ALLIANCE LOGIC
        if (aligning && turnIsBasicallyDone()) {
            follower.startTeleopDrive();
            aligning = false;
            telemetry.addLine("Turn tolerance override: teleop restored");
        }

        followerstuff();
        telemetry.addData("Alliance Blue?", bluealliance);

        Pose cur = follower.getPose();
        distanceToGoal = getDistance();
        telemetry.addData("distance to goal",distanceToGoal);
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("desired heading",Math.toDegrees(desiredHeading));
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
    private double getDistance(){
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal : redGoal;
        double hypotenuse,x,y;
        x = target.getX()-cur.getX();
        y= target.getY()-cur.getY();
        hypotenuse = Math.pow(x,2)+Math.pow(y,2);
        return Math.sqrt(hypotenuse);
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
        aligning2 = true;
    }
    private void goToHumanPlayer() {
        Pose target = bluealliance ? blueHP : redHP;

        if (pedroPose == null) return;

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(pedroPose, target))
                .setLinearHeadingInterpolation(pedroPose.getHeading(), target.getHeading())
                .build();

        follower.followPath(chain);
        aligning2 = true;
    }
    private void faceAllianceGoal() {
        Pose cur = follower.getPose();
        Pose target = bluealliance ? blueGoal : redGoal;
        if (pedroPose == null) return;

        double rawAngle = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        desiredHeading = flippedAngle;
        follower.turnTo(flippedAngle);
        aligning = true;
    }

    private void followerstuff() {
        follower.update();
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("AligningFlag", aligning);

        // ONLY exit aligning when Pedro says it's done (NO tolerance here)
        if (!follower.isBusy() && aligning2) {
            follower.startTeleopDrive();
            aligning2 = false;
            telemetry.addData("AlignStatus", "Finished - teleop re-enabled");
        } else if (aligning) {
            telemetry.addData("AlignStatus", "Running");
        }

        // Manual drive ONLY when idle AND NOT aligning
        if (!follower.isBusy() && !aligning) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speed,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                    -gamepad1.right_stick_x * speed,
                    true
            );
        }
    }





    private boolean turnIsBasicallyDone() {
        Pose cur = follower.getPose();
        double error = Math.abs(desiredHeading - cur.getHeading());
        error = Math.abs((error + Math.PI) % (2 * Math.PI) - Math.PI);
        return error < Math.toRadians(5);  // 5° tolerance → good for unsticking small turns
    }






}
