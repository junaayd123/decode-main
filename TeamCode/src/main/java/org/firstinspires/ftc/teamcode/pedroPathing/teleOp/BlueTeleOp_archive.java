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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretTest;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "archived blue teleop", group = "TeleOp")
public class BlueTeleOp_archive extends OpMode {
    private boolean aligning = false;
    private boolean aligning2 = false;
    private boolean alignForFar = false;
    private double distanceToGoal;
    // Coordinates of red/blue speaker tags (meters)

    private boolean bluealliance = false;
    private boolean blueStartPose = false;
    private double desiredHeading = 0;
    String motif = "gpp";

    public boolean tagInitializing;
    int[] ballsInRobot = {0,0,0};
    int greenInSlot;//0 if in left 1 if right, 2 if back
    private DcMotor intake = null;
    private Deposition_C depo;
    boolean shootingTest =false;
    boolean intakeRunning;
    Servo led;
    Servo led2;
    int lastShotSlot = -1; // 0 = Left, 1 = Right, 2 = Back, -1 = none
    private lifters LL;
    boolean direction = false; //false if intake is forward, true if depo;
    double speed;
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    Timer timer5;
    double ourVelo = 1300;
    boolean shooting = false;
    double shootinterval = 0.35;
    int shooterSequence;
    double timeOfSecondShot;

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2= new Gamepad();
    TurretLimelight turret;
    boolean alignToTags;

    private Follower follower;

    private final Pose startPose = new Pose(53,70,0); //red
    private final Pose blueGoal = new Pose(-72,140,0);
    private final Pose redGoal = new Pose(72,144,0);
    private final Pose blueGoalfar = new Pose(-69,144,0);
    private final Pose redGoalfar = new Pose(65,144,0);

    //below is all camera stuff
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Pose ftcPose, pedroPose;
    boolean tagDetected;
    double turretDeg;
    Timer turretTimer;
    double totalHedOffset;


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

            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            pedroPose = new Pose(ftcPose.getY(), -ftcPose.getX() + 72, ftcPose.getHeading()+Math.toRadians(turretDeg));
            break;
        }
    }
    double movingCase = 1;
    double timerDiddyMoment;
    public void moveDiddyTurret(){
        if(movingCase ==1){
            turret.setDegreesTarget(-180);
        }if(movingCase ==2){
            turret.setDegreesTarget(-150);
        }if(movingCase ==3){
            turret.setDegreesTarget(-120);
        }if(movingCase ==4){
            turret.setDegreesTarget(-90);
        }if(movingCase ==5){
            turret.setDegreesTarget(-60);
        }if(movingCase ==6){
            turret.setDegreesTarget(-30);
        }if(movingCase ==7){
            turret.setDegreesTarget(0);
        }if(movingCase ==8){
            turret.setDegreesTarget(30);
        }if(movingCase ==9){
            turret.setDegreesTarget(60);
        }if(movingCase ==10){
            turret.setDegreesTarget(90);
        }if(movingCase ==11){
            turret.setDegreesTarget(120);
        }if(movingCase ==12){
            turret.setDegreesTarget(150);
        }
        if(turretTimer.checkAtSeconds(0.5+timerDiddyMoment)){
            timerDiddyMoment = turretTimer.timer.seconds() - turretTimer.curtime;
            if(movingCase ==12){
                movingCase = 1;
            }
            else {
                movingCase += 1;
            }
        }
    }
    private enum Mode { nothing, findTag, faceGoal} //modes of turret
    private BlueTeleOp_archive.Mode mode = Mode.nothing;


    double headingTotag;
    boolean flywheelEarlyStart;
    @Override
    public void init() {
        turret = new TurretLimelight(hardwareMap);
        LL = new lifters(hardwareMap);
        depo = new Deposition_C(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        timer1 = new Timer();
        timer2 = new Timer();
        timer3 = new Timer();
        timer4 = new Timer();
        timer5 = new Timer();
        turretTimer = new Timer();
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        led = hardwareMap.get(Servo.class, "led");
        led2 = hardwareMap.get(Servo.class, "led2");
        initAprilTag();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    @Override
    public void start(){
        follower.startTeleopDrive();
        depo.setTargetVelocity(0);
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        timer3.resetTimer();
        timer4.resetTimer();
        timer5.resetTimer();
        turretTimer.resetTimer();
//        tagInitializing = true;
    }

    @Override
    public void loop() {
        Pose cur = follower.getPose();
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();
//        if(g1.psWasPressed()){
//            bluealliance = !bluealliance;
//            if(bluealliance) turret.setBlueAlliance();
//            else turret.setRedAlliance();
//        }
        follower.getTotalHeading();
//        if(g1.ps && g1.startWasPressed()){//invert pose
//            Pose invert = new Pose(-cur.getX(),cur.getY(),cur.getHeading()+Math.toRadians(180));
//            follower.setPose(invert);
//            blueStartPose = !blueStartPose;
//        }
        turret.updateEncoderPos();
        Pose targett;//used for turret alignment can be both red or blue and far or close
        Pose targett2;//used to calculate distanceToGoal can only be red or blue close
        if(distanceToGoal>125) {
            targett = blueGoalfar;
        }
        else{
            targett = blueGoal ;
        }
        targett2 = blueGoal;
        double rawAngle = Math.atan2(targett.getY() - cur.getY(), targett.getX() - cur.getX());

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        headingTotag = flippedAngle+Math.PI;
        double robHeading = follower.getTotalHeading()-totalHedOffset;
        while (robHeading >= Math.PI * 2 - Math.toRadians(-60)) robHeading -= 2 * Math.PI;
        while (robHeading < Math.toRadians(-60)) robHeading += 2 * Math.PI;

        if (gamepad2.rightBumperWasPressed()) {
            LL.allDown();
            if (intake.getPower() < -0.5) {
                intake.setPower(0);
                intakeRunning = false;
            } else {
                intake.setPower(-1);
                intakeRunning = true;
            }
        }
        if(shooting){
            intake.setPower(0);
            intakeRunning = false;
        }

        if (intakeRunning) {
            led.setPosition(0.28);
            led2.setPosition(0.28);
            if (LL.sensors.getRight() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getLeft() != 0) {
                timer3.startTimer();
                intakeRunning = false;
                led.setPosition(0.5);
                led2.setPosition(0.5);
            }
        }
        if(g2.left_bumper){
            intake.setPower(1);
        }
        else if(!g2.left_bumper && !intakeRunning && !timer3.timerIsOn()){
            intake.setPower(0);
        }

        reverseIntake();
        if(g1.cross) speed = 0.3;
        else speed = 1;
        if(g2.dpad_down && !preG2.dpad_down){
            shootingTest = !shootingTest;
        }
        if(g1.shareWasPressed()){
            turret.resetTurretEncoder();
        }
        if(g2.psWasPressed()){
            alignToTags = !alignToTags;
        }
        if(mode == Mode.faceGoal){
            turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingTotag));
        }
        if(mode == Mode.nothing){
            turret.TurretMotor.setPower(0);
        }
        if(mode == Mode.findTag){
            turret.toTargetInDegrees();
        }
        if (g1.triangleWasPressed()){
            if(tagInitializing){
                tagInitializing = false;
                led.setPosition(0);
                mode = Mode.nothing;
                pauseAprilTagDetection(); // Pause if canceling
            }
            else{
                turretTimer.startTimer();
                mode = Mode.findTag;
                turret.setDegreesTarget(0);
                led.setPosition(0.34);
                resumeAprilTagDetection(); // Resume when starting new detection
//                movingCase = 1;
//                timerDiddyMoment = 0;
            }
        }
        if(turretTimer.checkAtSeconds(1)){
            tagInitializing = true;
//            led.setPosition(0.34);
            turretTimer.stopTimer();
        }
        if (tagInitializing) {
            updateAprilTagLocalization();
            if (tagDetected && pedroPose != null) {
                mode = Mode.faceGoal;
                telemetry.addLine("seeing and localizing tag");
                telemetry.addData("local x",pedroPose.getX());
                telemetry.addData("local y",pedroPose.getY());
                telemetry.addData("local hed",Math.toDegrees(pedroPose.getHeading()));
                telemetry.addData("turret angle",turretDeg);
                follower.setPose(pedroPose.getPose());
                totalHedOffset = follower.getTotalHeading()-pedroPose.getHeading();
                tagInitializing = false;
                led.setPosition(0.6);
                pauseAprilTagDetection();
            }
            else{

            }
        }
        if(distanceToGoal>125) shootinterval = 0.4;
        else shootinterval = 0.35;
        if(g2.cross && !preG2.cross){//shoot 3 close
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                }
                else {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
                }
                motif = "gpp";
                greenInSlot = 0;
                shooting = true;

            }
        }
        if(!shootingTest){
            if(shooting) depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
            LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
            if(flywheelEarlyStart) {
                if (LL.sensors.getLeft() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getRight() != 0) {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                }
                else if(!shooting && timer1.timerIsOff()){
                    depo.setTargetVelocity(0);
                }
            }
        }
        if(g2.dpadLeftWasPressed()){
            timer1.stopTimer();
            intakeRunning = false;
            shooting = false;
            intake.setPower(0);
            depo.setTargetVelocity(0);
            LL.allDown();
        }

        if(g2.square && !preG2.square){//gpp
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                }
                else {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
                }
                shooting = true;
                motif = "gpp";
                greenInSlot = getGreenPos();
                ballsInRobot[0] = LL.sensors.getLeft();
                ballsInRobot[1] = LL.sensors.getRight();
                ballsInRobot[2] = LL.sensors.getBack();

            }
        }
        if(g2.triangle && !preG2.triangle){//pgp
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                }
                else {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
                }
                shooting = true;
                motif = "pgp";
                greenInSlot = getGreenPos();
                ballsInRobot[0] = LL.sensors.getLeft();
                ballsInRobot[1] = LL.sensors.getRight();
                ballsInRobot[2] = LL.sensors.getBack();

            }
        }
        if(g2.circle && !preG2.circle){//ppg
            if(!LL.checkNoBalls()) {
                if(shootingTest){
                    depo.setTargetVelocity(ourVelo);
                }
                else {
                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
                }
                shooting = true;
                motif = "ppg";
                greenInSlot = getGreenPos();
                ballsInRobot[0] = LL.sensors.getLeft();
                ballsInRobot[1] = LL.sensors.getRight();
                ballsInRobot[2] = LL.sensors.getBack();

            }
        }


        // SMALL TURN UNSTICKING ONLY FOR FACE-ALLIANCE LOGIC

        followerstuff();
        telemetry.addData("Alliance Blue?", bluealliance);
        telemetry.addData("start pose Blue?", blueStartPose);
        distanceToGoal = cur.distanceFrom(targett2);// used to be getDistance();
        telemetry.addData("turret tick pos",turret.currentPos);
        telemetry.addData("align to tag",alignToTags);
        telemetry.addData("shooter sequence",shooterSequence);
        telemetry.addData("actual depo velo",depo.getVelocity());
        telemetry.addLine(shootingTest ? "Testing shooting using cross":"regular teleOp shooting");
        telemetry.addData("distance to goal",distanceToGoal);
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("shooting angle", LL.launchAngleServo.getPosition());
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading()-totalHedOffset));
        telemetry.addData("desired heading",Math.toDegrees(desiredHeading));
        if(depo.reachedTargetHighTolerance()){
            if (shooting) {
                // Now start timer to shoot the 3 balls
                timer1.startTimer();
                shooting = false;
            }
        }
        if(motif.equals("gpp")){
            if(greenInSlot == 0) LRBnoRecovery();
            else if(greenInSlot == 1) RBLnoRecovery();
            else BLRnoRecovery();
        }
        else if(motif.equals("pgp")){
            if(greenInSlot == 0) BLRnoRecovery();
            else if(greenInSlot == 1) LRBnoRecovery();
            else RBLnoRecovery();
        }
        else{
            if(greenInSlot == 0) RBLnoRecovery();
            else if(greenInSlot == 1) BLRnoRecovery();
            else LRBnoRecovery();
        }

        if(g1.dpad_up&& !preG1.dpad_up){
            ourVelo+=25;
        }
        else if(g1.dpad_down&& !preG1.dpad_down){
            ourVelo-=25;
        }
        if(g1.dpad_left&& !preG1.dpad_left){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()-0.03);
        }
        else if(g1.dpad_right&& !preG1.dpad_right){
            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()+0.03);
        }

        // Telemetry

        telemetry.update();
    }
    private void ledColorTime(double color, double sec){
        if(timer5.checkAtSeconds(0)){
            led.setPosition(color);
        }
        if(timer5.checkAtSeconds(sec)){
            led.setPosition(0);
            timer5.stopTimer();
        }
    }
    //b
    private int veloBasedOnDistance(double dist){
        //https://www.desmos.com/calculator/nxghj961jg
        //desmos table for the shooting velo
        //x is distanceCM y1 is velo y2 is launch angle
        //below is old stuff
        if(dist<125){
            return (int) (5.35158*dist+873.83526);
        }//(3.69593*dist+960.60458); old
        else
            return (int) (7.14286*(7+dist)+589.28571); //far//far
//        if(!bluealliance) {
//            if (dist < 60) return 1125; //close distance
//            else if (dist < 70) return 1150;
//            else if (dist < 75) return 1175;
//            else if (dist < 80) return 1200;
//            else if (dist < 87) return 1225;
//            else if (dist < 110) return 1300;
//            else if (dist > 115 && dist < 150) return 1480;//far distance
//            else return 0;//didnt localize the tag
//        }
//        else{
//            if (dist+5 < 60) return 1125; //close distance
//            else if (dist+5 < 70) return 1150;
//            else if (dist+5 < 75) return 1175;
//            else if (dist+5 < 80) return 1200;
//            else if (dist+5 < 87) return 1225;
//            else if (dist+5 < 110) return 1300;
//            else if (dist+5 > 115 && dist < 150) return 1480;//far distance
//            else return 0;//didnt localize the tag
//        }
    }
    private double angleBasedOnDistance(double dist){
//        if(dist<70) return 0.06; //close distance
//        else if(dist<87) return 0.09; //close distance
//        else if(dist<110) return 0.12; //close distance
//        else if(dist>115 && dist<150) return 0.18;//far distance
//        else return 0.06; //this shouldnt happen but 0.06 is a safe backup
        if (dist>125) return 0.27;
        else return 0.00180592*dist-0.0205829;
        //old 0.00132566*dist+0.00291356
    }
    private void reverseIntake() {
        if (timer3.checkAtSeconds(0)) {
            intake.setPower(1);
        }
        if (timer3.checkAtSeconds(0.5)) {
            intake.setPower(0);
            timer3.stopTimer();
        }
    }

    // ---------- APRILTAG UPDATE (for triangle-held localization) ----------

    // ---------- DRIVE TO RED OR BLUE SHOOTING POSE ----------

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
            if(direction) {
//                led.setPosition(0.388);
                follower.setTeleOpDrive( gamepad1.left_stick_y*speed, (gamepad1.right_trigger - gamepad1.left_trigger)*speed, -gamepad1.right_stick_x*speed, true );
            }
            else {
//                led.setPosition(0);
                follower.setTeleOpDrive( -gamepad1.left_stick_y*speed, (gamepad1.left_trigger - gamepad1.right_trigger)*speed, -gamepad1.right_stick_x*speed, true );
            }
        }
    }





    private int getGreenPos(){
        int pos;
        pos = LL.sensors.getLeft();
        if(pos==1) return 0;
        else{
            pos = LL.sensors.getRight();
            if(pos==1) return 1;
            else return 2;
        }
    }
    private void LRBnoRecovery(){
        if (timer1.checkAtSeconds(0)) {
            LL.leftUp();
            shooterSequence = 1;
        }
        if(timer1.checkAtSeconds(shootinterval) && shooterSequence==1){
            LL.allDown();
            LL.rightUp();
            shooterSequence = 2;
        }
        if(timer1.checkAtSeconds(shootinterval*2) && shooterSequence==2){
            LL.allDown();
            LL.backUp();
            shooterSequence = 3;
        }
        if(timer1.checkAtSeconds(shootinterval*3) && shooterSequence==3){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }
    private void BLRnoRecovery(){
        if (timer1.checkAtSeconds(0)) {
            LL.backUp();
            shooterSequence = 1;
        }
        if(timer1.checkAtSeconds(shootinterval) && shooterSequence==1){
            LL.allDown();
            LL.leftUp();
            shooterSequence = 2;
        }
        if(timer1.checkAtSeconds(shootinterval*2) && shooterSequence==2){
            LL.allDown();
            LL.rightUp();
            shooterSequence = 3;
        }
        if(timer1.checkAtSeconds(shootinterval*3) && shooterSequence==3){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }
    private void RBLnoRecovery(){
        if (timer1.checkAtSeconds(0)) {
            LL.rightUp();
            shooterSequence = 1;
        }
        if(timer1.checkAtSeconds(shootinterval) && shooterSequence==1){
            LL.allDown();
            LL.backUp();
            shooterSequence = 2;
        }
        if(timer1.checkAtSeconds(shootinterval*2) && shooterSequence==2){
            LL.allDown();
            LL.leftUp();
            shooterSequence = 3;
        }
        if(timer1.checkAtSeconds(shootinterval*3) && shooterSequence==3){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
        }
    }

    private void shootLRB() {//shoots in left right back order
//        if (lastShotSlot == -1) return; // nothing scheduled

        if (timer1.checkAtSeconds(0)) { //this executes when depo reached target so timer just started and we can fire the first shot
//            fireShotFromSlot(lastShotSlot); //lifts the first ball
            LL.leftUp();
            shooterSequence = 1; //this variable is a flag for the sequence to run properly
        }

        // Shot 2
        if (timer1.checkAtSeconds(shootinterval)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
            LL.allDown();
            shooterSequence = 2;//sets up to check the depo velocity again
        }
        if(shooterSequence==2 && depo.reachedTargetHighTolerance()){ //this if statement is ran after depo reached target
//            fireNextAvailableShot();//lifts second ball
            LL.rightUp();
            shooterSequence=3;//sets the sequence to check
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;//gets the curent time of the sequence so that next block runs now+0.4 instead of at a 0.8 seconds
        }

        // Shot 3
        if (timer1.checkAtSeconds(shootinterval+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
            LL.allDown();//puts the lifts down
            shooterSequence = 4;
        }
        if(shooterSequence==4 && depo.reachedTargetHighTolerance()){//does the velocity check again
            LL.backUp();
//            fireNextAvailableShot();
            shooterSequence=5;
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;
        }

        // Finish cycle
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==5) {//resets the whole timer and sequence is done
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
            lastShotSlot = -1; // ✅ CONSUMES SLOT — will NOT shoot same one again
        }
    }
    private void shootBLR(){//shoots in back left right order
//        if (lastShotSlot == -1) return; // nothing scheduled

        if (timer1.checkAtSeconds(0)) { //this executes when depo reached target so timer just started and we can fire the first shot
//            fireShotFromSlot(lastShotSlot); //lifts the first ball
            LL.backUp();
            shooterSequence = 1; //this variable is a flag for the sequence to run properly
        }

        // Shot 2
        if (timer1.checkAtSeconds(shootinterval)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
            LL.allDown();
            shooterSequence = 2;//sets up to check the depo velocity again
        }
        if(shooterSequence==2 && depo.reachedTargetHighTolerance()){ //this if statement is ran after depo reached target
//            fireNextAvailableShot();//lifts second ball
            LL.leftUp();
            shooterSequence=3;//sets the sequence to check
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;//gets the curent time of the sequence so that next block runs now+0.4 instead of at a 0.8 seconds
        }

        // Shot 3
        if (timer1.checkAtSeconds(shootinterval+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
            LL.allDown();//puts the lifts down
            shooterSequence = 4;
        }
        if(shooterSequence==4 && depo.reachedTargetHighTolerance()){//does the velocity check again
            LL.rightUp();
//            fireNextAvailableShot();
            shooterSequence=5;
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;
        }

        // Finish cycle
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==5) {//resets the whole timer and sequence is done
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
            lastShotSlot = -1; // ✅ CONSUMES SLOT — will NOT shoot same one again
        }
    }
    private void shootRBL(){//shoots in right back left order
//        if (lastShotSlot == -1) return; // nothing scheduled

        if (timer1.checkAtSeconds(0)) { //this executes when depo reached target so timer just started and we can fire the first shot
//            fireShotFromSlot(lastShotSlot); //lifts the first ball
            LL.rightUp();
            shooterSequence = 1; //this variable is a flag for the sequence to run properly
        }

        // Shot 2
        if (timer1.checkAtSeconds(shootinterval)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
            LL.allDown();
            shooterSequence = 2;//sets up to check the depo velocity again
        }
        if(shooterSequence==2 && depo.reachedTargetHighTolerance()){ //this if statement is ran after depo reached target
//            fireNextAvailableShot();//lifts second ball
            LL.backUp();
            shooterSequence=3;//sets the sequence to check
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;//gets the curent time of the sequence so that next block runs now+0.4 instead of at a 0.8 seconds
        }

        // Shot 3
        if (timer1.checkAtSeconds(shootinterval+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
            LL.allDown();//puts the lifts down
            shooterSequence = 4;
        }
        if(shooterSequence==4 && depo.reachedTargetHighTolerance()){//does the velocity check again
            LL.leftUp();
//            fireNextAvailableShot();
            shooterSequence=5;
            timeOfSecondShot = timer1.timer.seconds()-timer1.curtime;
        }

        // Finish cycle
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==5) {//resets the whole timer and sequence is done
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
            shooterSequence = 0;
            lastShotSlot = -1; // ✅ CONSUMES SLOT — will NOT shoot same one again
        }
    }
    private void pauseAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    // Add this method to resume detection
    private void resumeAprilTagDetection() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

}

//hi