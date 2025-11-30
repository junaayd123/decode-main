package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Lm2 2 drivers", group = "TeleOp")
public class teleOplm2 extends OpMode {
    private boolean aligning = false;
    private boolean aligning2 = false;
    private double distanceToGoal;
    // Coordinates of red/blue speaker tags (meters)

    private boolean bluealliance = false;
    private double desiredHeading = 0;



    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(320.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(220.0));
    private final Pose redGoal  = new Pose(130, 130, 0);
    private final Pose blueGoal  = new Pose(14, 130,0);
    private final Pose redGoal2  = new Pose(144, 144, 0);
    private final Pose blueGoal2  = new Pose(0, 144,0);
    private final Pose redHP  = new Pose(42, 25, Math.toRadians(180)); //red human player
    private final Pose blueHP  = new Pose(115, 25,0); // blue human player

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public boolean tagDetected;

    Pose pedroPose, ftcPose;
    private DcMotor intake = null;
    private Deposition depo;
    Servo led;
    int lastShotSlot = -1; // 0 = Left, 1 = Right, 2 = Back, -1 = none

    private lift_three LL;
    boolean direction = false; //false if intake is forward, true if depo;
    double speed;
    Timer timer1;
    Timer timer2;
    double ourVelo = 1300;
    boolean shooting = false;
    boolean shooting2 = false;
    boolean greenball = false;//false is purp true is geren

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2= new Gamepad();

    private Follower follower;

    private final Pose startPose = new Pose(0,0,0);
    @Override
    public void init() {
        LL = new lift_three(hardwareMap);
        depo = new Deposition(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        timer1 = new Timer();
        timer2 = new Timer();
        initAprilTag();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        led = hardwareMap.get(Servo.class, "led");
        // Optional: set directions if your motors are mounted opposite each other
        // intake.setDirection(DcMotorSimple.Direction.FORWARD);
        // depo.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        depo.setTargetVelocity(0);
        depo.kF = -0.00048;
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();

    }

    @Override
    public void loop() {
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();
        if(g1.psWasPressed()) bluealliance = !bluealliance;
        if(g2.right_bumper){
            intake.setPower(-1);
        } else if (g2.left_bumper) {
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
        if(g1.cross) speed = 0.3;
        else speed = 1;
        if(g2.cross && !preG2.cross){//shoot 3 close
            if(!LL.checkNoBalls()) {
//                depo.setTargetVelocity(depo.closeVelo_New);
//                LL.set_angle_close();
                depo.setTargetVelocity(ourVelo);
                shooting = true;
            }
        }
        if(g2.square && !preG2.square){//shoot purp
            depo.setTargetVelocity(depo.closeVelo_New);
            LL.set_angle_close();
            greenball = false;
            shooting2 = true;
        }
        if(g2.circle && !preG2.circle){//shoot green
            depo.setTargetVelocity(depo.closeVelo_New);
            LL.set_angle_close();
            greenball = true;
            shooting2 = true;
        }
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
        if(g2.triangle && !preG2.triangle){//shoot far
            if(!LL.checkNoBalls()) {
                depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                if(distanceToGoal>120) LL.set_angle_far();
                else LL.set_angle_close();
                shooting = true;
            }
        }
        if(depo.reachedTarget()){
            if (shooting) {
                // Decide which slot will actually shoot
                if (LL.sensors.getLeft() != 0 && lastShotSlot != 0) {
                    lastShotSlot = 0;
                } else if (LL.sensors.getRight() != 0 && lastShotSlot != 1) {
                    lastShotSlot = 1;
                } else if (LL.sensors.getBack() != 0 && lastShotSlot != 2) {
                    lastShotSlot = 2;
                } else {
                    // No valid new balls → cancel shooting
                    depo.setTargetVelocity(0);
                    shooting = false;
                    lastShotSlot = -1;
                    return;
                }

                // Now start timer to shoot the 3 balls
                timer1.startTimer();
                shooting = false;
            }
            if(shooting2){
                timer2.startTimer();
                shooting2 = false;
            }
        }
        shoot3x();
        shootoneColored();
        if(g1.dpad_up&& !preG1.dpad_up){
            ourVelo+=50;
        }
        else if(g1.dpad_down&& !preG1.dpad_down){
            ourVelo-=50;
        }
        if(g1.dpad_left&& !preG1.dpad_left){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()-0.03);
        }
        else if(g1.dpad_right&& !preG1.dpad_right){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()+0.03);
        }

        // Telemetry
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("curent angle", LL.launchAngleServo.getPosition());
        telemetry.update();
    }
    private int veloBasedOnDistance(double dist){
        if(dist<55) return 1100;
        else if(dist<70) return 1150;
        else if(dist<80) return 1230;
        else if(dist<110) return 1350;
        else if(dist<140) return 1600;
        else return 1650;
    }
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
        Pose target = bluealliance ? blueGoal2 : redGoal2;
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
        return error < Math.toRadians(2);  // 5° tolerance → good for unsticking small turns
    }
    private void shoot3x() {
        if (lastShotSlot == -1) return; // nothing scheduled

        // Shot 1 fire
        if (timer1.checkAtSeconds(0)) {
            fireShotFromSlot(lastShotSlot);
        }

        // Shot 2
        if (timer1.checkAtSeconds(0.3)) {
            LL.allDown();
            fireNextAvailableShot();
        }

        // Shot 3
        if (timer1.checkAtSeconds(0.6)) {
            LL.allDown();
            fireNextAvailableShot();
        }

        // Finish cycle
        if (timer1.checkAtSeconds(0.9)) {
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            lastShotSlot = -1; // ✅ CONSUMES SLOT — will NOT shoot same one again
        }
    }
    private void fireShotFromSlot(int slot) {
        if (slot == 0) LL.leftUp();
        if (slot == 1) LL.rightUp();
        if (slot == 2) LL.backUp();
    }

    private void fireNextAvailableShot() {
        // pick a NEW slot that still has balls and wasn't already shot
        if (LL.sensors.getLeft() != 0 && lastShotSlot != 0) {
            LL.leftUp();
            lastShotSlot = 0;
        }
        else if (LL.sensors.getRight() != 0 && lastShotSlot != 1) {
            LL.rightUp();
            lastShotSlot = 1;
        }
        else if (LL.sensors.getBack() != 0 && lastShotSlot != 2) {
            LL.backUp();
            lastShotSlot = 2;
        }
        else {
            // no other balls → end cycle early
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            LL.allDown();
            lastShotSlot = -1;
        }
    }

    private void shootoneColored(){
        if(timer2.checkAtSeconds(0)){
            if(greenball){
                LL.lift_green();
            }
            else LL.lift_purple();
        }
        if(timer2.checkAtSeconds(0.3)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
        }
    }

}