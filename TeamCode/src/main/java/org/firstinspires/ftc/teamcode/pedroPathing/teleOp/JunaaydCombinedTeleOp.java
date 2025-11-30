package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

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
//import com.pedropathing.ftc.FTCCoordinates;

import java.util.List;

@TeleOp(name = "junaayd combined teleop", group = "TeleOp")
public class JunaaydCombinedTeleOp extends OpMode {
    private boolean aligning = false;   // true while the robot is following the align path

    private DcMotor intake = null;
    private Deposition depo;
    private boolean bluealliance = false;
    Gamepad preG1= new Gamepad();
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    Timer timer5;

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad g2= new Gamepad();
    private launch_lift LL;
    private double speed = 1;

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0.0);
    private final Pose blueNearShootPose = new Pose(50,  100, Math.toRadians(135.0));
    private final Pose redNearShootPose = new Pose(94,  100, Math.toRadians(45.0));

    // === AprilTag / Vision members ===
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public boolean tagDetected;
    public boolean pauseTagDetection;
    public boolean followPathSquarePressed;
    Pose pedroPose, ftcPose;

    @Override
    public void init() {
        // ----- existing init -----
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

        // ----- turning on pause tag detection -----
        pauseTagDetection=true;

        // ----- aprilTag init -----
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start(){
        follower.startTeleopDrive();
        LL.down();
        LL.close();
        timer1.resetTimer();
        timer2.resetTimer();
        timer3.resetTimer();
        timer4.resetTimer();
        timer5.resetTimer();
        depo.setTargetVelocity(0);
    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        depo.updatePID();
        //intake with gampad cross
        if(g1.cross){
            speed = 0.3;
        }
        else{speed=1;}
        boolean allTimersOff = !timer1.timerIsOn() && !timer2.timerIsOn() && !timer3.timerIsOn() && !timer4.timerIsOn() && !timer5.timerIsOn();
        if (g2.right_bumper && allTimersOff) {
            intake.setPower(-1.0);
        }else if (g2.left_bumper && !preG2.left_bumper && allTimersOff){
            timer5.startTimer();
        }
        else if(!timer1.timerIsOn() && !timer2.timerIsOn() && allTimersOff) {
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
        if(g1.square && !preG1.square){
            //align_far_red(bluealliance);
        }
        if (g1.squareWasPressed()) {
            followPathSquarePressed = true;
        }
        if (g1.triangle) {
            pauseTagDetection=false;
        }
        if(g1.ps && !preG1.ps){
            bluealliance = !bluealliance;
        }

        // Depo with gamepad circle
        if (g2.circle && !preG2.circle && !g2.start) {
            timer2.startTimer();
        }
        if (g2.square && !preG2.square){
            timer1.startTimer();
        }
        if(g2.triangle && !preG2.triangle){
            timer3.stopTimer();
        }
        if(g2.cross && !preG2.cross){
            timer4.startTimer();
        }

        farshoot3x();
        closeshoot3x();
        farshoot();
        closeshoot();
        spitOut();

        // ---------- update april tags and apply to follower ----------
        updateAprilTagPedro();

        // if we have a valid tag-derived pedroPose, update follower pose
        if (!pauseTagDetection && !follower.isBusy() && tagDetected && pedroPose != null) {
            follower.setPose(pedroPose.getPose());
        }

        followerstuff();

        // Telemetry (existing)
        telemetry.addData("is alliance blue?",bluealliance);
        telemetry.addData("Launch Position", LL.launchServo.getPosition());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Depo Power", depo.right.getPower());
        telemetry.update();
    }

    // ---------------- AprilTag init ----------------
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
                telemetry.addLine("Warning: Webcam 'Webcam 1' not found");
            }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Add the processor and build.
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    // ---------------- update april tag detections and telemetry ----------------
    private void updateAprilTagPedro() {
        if (aprilTag == null) return;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        tagDetected = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagDetected = true; // Tag that is good for tracking LOCATION ON FIELD

                    double xInches = detection.robotPose.getPosition().x; // already in inches
                    double yInches = detection.robotPose.getPosition().y;
                    double headingDeg = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    // Construct an ftcPose in FTCCoordinates
                    ftcPose = new Pose(xInches, yInches, Math.toRadians(headingDeg));

                    // Convert to Pedro coordinates
                    pedroPose = new Pose(ftcPose.getY() + 72, -ftcPose.getX() + 72, ftcPose.getHeading());

                    // bearing from detection (how much we need to rotate so tag faces camera) - provided by sdk
                    double bearing = detection.ftcPose.bearing;
                    double yawTagField = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                    double desiredHeadingDeg = yawTagField + bearing;

                    // edge-detected button handling for follower.turnTo() and break
                    if (g1.circle && !preG1.circle && tagDetected && !follower.isBusy()) {
                        //follower.turnTo(Math.toRadians(desiredHeadingDeg));
                        pauseTagDetection = true;
                        PathChain blueShoot = follower.pathBuilder()
                                .addPath(new BezierLine(pedroPose, blueNearShootPose))
                                .setLinearHeadingInterpolation(pedroPose.getHeading(), blueNearShootPose.getHeading())
                                .build();
                        PathChain redShoot = follower.pathBuilder()
                                .addPath(new BezierLine(pedroPose, redNearShootPose))
                                .setLinearHeadingInterpolation(pedroPose.getHeading(), redNearShootPose.getHeading())
                                .build();
                        if (followPathSquarePressed) {
                            followPathSquarePressed = false;
                            if (bluealliance) {
                                follower.followPath(blueShoot);
                            } else {
                                follower.followPath(redShoot);
                            }
                        }
                    }
                    if (g1.square && !preG1.square && tagDetected && follower.isBusy()) {
                        //follower.breakFollowing();
                        //pauseTagDetection = true;
                    }

                    // telemetry (detection & follower debug)
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addData("Bearing", detection.ftcPose.bearing);
                    telemetry.addData("Degrees to turn", desiredHeadingDeg);
                    telemetry.addData("is it turning", follower.isTurning());
                    telemetry.addData("is tag detection paused?", pauseTagDetection);
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for()

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    private void align_far_red(boolean blue) {
        double angle;
        if (blue) {
            angle = 22.5;
        } else {
            angle = -22.5;
        }
        Pose cur = follower.getPose();
        Pose align = new Pose(cur.getX() -Math.cos(angle)*4, cur.getY()-Math.sin(angle)*4, cur.getHeading() + Math.toRadians(angle));

        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, align)))
                .setLinearHeadingInterpolation(cur.getHeading(), align.getHeading())
                .build();

        // start path asynchronously
        follower.followPath(first, false);
        aligning = true;

        // optional immediate update so follower starts processing now (harmless)
        follower.update();
    }
    public void spitOut(){
        if(timer5.checkAtSeconds(0)) {
            intake.setPower(0.3);
        }
        if(timer5.checkAtSeconds(0.1)) {
            timer5.stopTimer();
        }
    }
    public void farshoot(){
        if(timer1.checkAtSeconds(0)){
            depo.setTargetVelocity(depo.farVelo);
            LL.far();
        }
        if(timer1.checkAtSeconds(0.5)){
            LL.up();
        }
        if (timer1.checkAtSeconds(0.8)){
            LL.down();
            depo.setTargetVelocity(0);
            timer1.stopTimer();

        }
    }


    public void closeshoot(){

        if(timer2.checkAtSeconds(0)){
            depo.setTargetVelocity(depo.closeVelo);
            LL.close();
        }
        if(timer2.checkAtSeconds(0.5)){
            LL.up();
        }
        if (timer2.checkAtSeconds(0.8)){
            LL.down();
            depo.setTargetVelocity(0);
            timer2.stopTimer();

        }
    }
    public void farshoot3x() {
        // shot 1
        if (timer3.checkAtSeconds(0)) {
            depo.setTargetVelocity(depo.farVelo);
            LL.far();
        }
        if (timer3.checkAtSeconds(0.5)) LL.up();
        if (timer3.checkAtSeconds(0.8)) LL.down();

        // feed 1 and slightly bump power if you used to
        if (timer3.checkAtSeconds(1.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 2
        if (timer3.checkAtSeconds(1.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer3.checkAtSeconds(1.8)) LL.down();

        // feed 2 + return to main FAR power
        if (timer3.checkAtSeconds(2.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 3
        if (timer3.checkAtSeconds(2.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer3.checkAtSeconds(2.8)) {
            LL.down();
            depo.setTargetVelocity(0);
            timer3.stopTimer();
            pauseTagDetection = false;
        }
    }

    public void closeshoot3x() {
        // shot 1
        if (timer4.checkAtSeconds(0)) {
            depo.setTargetVelocity(depo.closeVelo);
            LL.close();
        }
        if (timer4.checkAtSeconds(0.5)) LL.up();
        if (timer4.checkAtSeconds(0.8)) LL.down();

        // feed 1 and slightly bump power if you used to
        if (timer4.checkAtSeconds(1.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 2
        if (timer4.checkAtSeconds(1.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer4.checkAtSeconds(1.8)) LL.down();

        // feed 2 + return to main FAR power
        if (timer4.checkAtSeconds(2.0)) {
            if (intake != null) intake.setPower(-1);
        }

        // shot 3
        if (timer4.checkAtSeconds(2.5)) {
            LL.up();
            if (intake != null) intake.setPower(0);
        }
        if (timer4.checkAtSeconds(2.8)) {
            LL.down();
            depo.setTargetVelocity(0);
            timer4.stopTimer();

        }
    }
    private void followerstuff(){
        // Update follower first so it can progress following path if active
        follower.update();

        // Telemetry & debug info about follower state
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("AligningFlag", aligning);

        // If we are currently aligning, wait for follower to finish and then return to teleop
        if (aligning) {
            if (!follower.isBusy()) {
                // path finished
                follower.startTeleopDrive();  // re-enable teleop drive mode in follower
                aligning = false;
                telemetry.addData("AlignStatus", "Finished - teleop re-enabled");
            } else {
                // follower still busy — optionally skip applying driver stick control until finished
                telemetry.addData("AlignStatus", "Running");
            }
        }

        // Only set teleop movement vectors when follower is NOT busy (i.e., manual control allowed)
        if (!follower.isBusy()) {
            // apply driver joystick input
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * speed,
                    (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
                    -gamepad1.right_stick_x * speed,
                    true
            );
        } else {
            // optional: zero sticks to make behavior predictable while follower drives
            // follower.setTeleOpMovementVectors(0, 0, 0, true);
        }
    }
}
