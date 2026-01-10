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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Bot C teleop", group = "TeleOp")
public class BotCTeleop extends OpMode {
    private boolean aligning = false;
    private boolean aligning2 = false;
    private boolean alignForFar = false;
    private double distanceToGoal;
    // Coordinates of red/blue speaker tags (meters)

    private boolean bluealliance = false;
    private boolean blueStartPose = false;
    private double desiredHeading = 0;
    String motif = "gpp";




    private final Pose blueNearShootPose = new Pose(50, 100, Math.toRadians(320.0));
    private final Pose redNearShootPose  = new Pose(94, 100, Math.toRadians(220.0));
    private final Pose blueFarShootPose = new Pose(65, 25, Math.toRadians(-61));
    private final Pose redFarShootPose  = new Pose(80, 25, Math.toRadians(-115));
//    private final Pose redGoal2  = new Pose(144, 144, 0);
//    private final Pose blueGoal2  = new Pose(0, 144,0);
    private final Pose redHP  = new Pose(42, 25, Math.toRadians(180)); //red human player
    private final Pose blueHP  = new Pose(115, 25,0); // blue human player

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);

    public boolean tagDetected;
    public boolean tagInitializing;
    String shotSlot1;
    String shotSlot2;
    String shotSlot3;


    Pose pedroPose, ftcPose;
    int[] ballsInRobot = {0,0,0};
    int greenInSlot;//0 if in left 1 if right, 2 if back
    private DcMotor intake = null;
    private Deposition_C depo;
    boolean shootingTest =false;
    boolean intakeRunning;
    Servo led;
    int lastShotSlot = -1; // 0 = Left, 1 = Right, 2 = Back, -1 = none
    private lifters LL;
    boolean direction = false; //false if intake is forward, true if depo;
    double speed;
    Timer timer1;
    Timer timer2;
    Timer timer3;
    Timer timer4;
    double ledRunTime;
    double ledColor;
    Timer timer5;
    double ourVelo = 1300;
    boolean shooting = false;
    boolean shooting2 = false;
    double shootinterval = 0.35;
    boolean hasppg;//true if robot holds pu
    boolean rightFlip = true;//true if hasnt flipped yet
    boolean leftFlip = true;//true if hasnt flipped yet
    boolean backFlip = true;//true if hasnt flipped yet
    int shooterSequence;
    double timeOfSecondShot;
    int shootingHasWorked = -1;
    boolean shootingHasWorkedNoVelo;
    boolean greenball = false;//false is purp true is geren

    Gamepad g1= new Gamepad();
    Gamepad preG2= new Gamepad();
    Gamepad preG1 = new Gamepad();
    Gamepad g2= new Gamepad();
    TurretLimelight turret;
    boolean alignToTags;

    private Follower follower;

    private final Pose startPose = new Pose(53,70,0); //red
    private final Pose blueGoal = new Pose(-72,140,0);
    private final Pose redGoal = new Pose(72,140,0);
    private final Pose blueGoalfar = new Pose(-68,144,0);
    private final Pose redGoalfar = new Pose(68,144,0);
    double headingTotag;
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
//        turret.InitLimelight();
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
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        timer3.resetTimer();
        timer4.resetTimer();
        timer5.resetTimer();
        tagInitializing = true;
    }

    @Override
    public void loop() {
        Pose cur = follower.getPose();
        preG1.copy(g1);
        preG2.copy(g2);
        g1.copy(gamepad1);
        g2.copy(gamepad2);
        depo.updatePID();
        if(g1.psWasPressed()){
            bluealliance = !bluealliance;
            if(bluealliance) turret.setBlueAlliance();
            else turret.setRedAlliance();
        }
        if(g1.ps && g1.startWasPressed()){//invert pose
            Pose invert = new Pose(-cur.getX(),cur.getY(),cur.getHeading()+Math.toRadians(180));
            follower.setPose(invert);
            blueStartPose = !blueStartPose;
        }
//        turret.updateLimelight();
        turret.updateEncoderPos();
        Pose targett;
        if(distanceToGoal>120) {
            targett = bluealliance ? blueGoalfar : redGoalfar;
        }
        else{
            targett = bluealliance ? blueGoal : redGoal;
        }
        double rawAngle = Math.atan2(targett.getY() - cur.getY(), targett.getX() - cur.getX());

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        headingTotag = flippedAngle+Math.PI;

        if (gamepad2.rightBumperWasPressed()) {
            if (intake.getPower() < -0.5) {
                intake.setPower(0);
                intakeRunning = false;
            } else {
                intake.setPower(-1);
                intakeRunning = true;
            }
        }

        if (intakeRunning) {
            if (LL.sensors.getRight() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getLeft() != 0) {
                timer3.startTimer();
                intakeRunning = false;
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
        if(alignToTags){
//            telemetry.addLine("trying to do the turret bs");
            turret.toTargetInDegrees2(Math.toDegrees(cur.getHeading() - headingTotag));
        }
        else {
//            telemetry.addLine("setting turret power to 0");
            turret.TurretMotor.setPower(0);
        }
        if(distanceToGoal>120) shootinterval = 0.4;
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
        if(!shootingTest && shooting){
            depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
            LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
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
        if (g1.triangle && !preG1.triangle){
            tagInitializing = true;
        }
        if (tagInitializing) {
//            updateAprilTagLocalization();
//            if (tagDetected && pedroPose != null && !follower.isBusy()) {
//                follower.setPose(pedroPose.getPose());
//                ledColor = 0.5;//green
//                ledRunTime = 0.5;
//                timer5.startTimer();
//                tagInitializing = false;
//            }
//            else{
//                led.setPosition(0.388);//yellow
//            }
        }


        // ========= WHEN SQUARE PRESSED → DRIVE TO SHOOTING POSE =========
        if (g1.square && !preG1.square) {
//            if (!follower.isBusy()) {
//                goToHumanPlayer();
//            }
            direction = !direction;
        }
//        if (g1.circle && !preG1.circle && !follower.isBusy()) {
//            if(distanceToGoal<115) {
//                faceAllianceGoal();
//                timer4.startTimer();//lets allignment run for 1 sec
//            }
//            else{
//                goToFarPose();
//                alignForFar=true;
//            }
//        }
        if(alignForFar && !follower.isBusy()){
            timer4.startTimer();
            alignForFar = false;
        }
        quitCorrectingAngle();
        if (g1.dpad_down && !preG1.dpad_down) {
            follower.breakFollowing();      // stops all paths and turns
            follower.startTeleopDrive();    // force drive mode back
            aligning = false;              // clear flags
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
        }
        // SMALL TURN UNSTICKING ONLY FOR FACE-ALLIANCE LOGIC

        followerstuff();
        telemetry.addData("Alliance Blue?", bluealliance);
        telemetry.addData("start pose Blue?", blueStartPose);
        ledColorTime(ledColor, ledRunTime);
//        Pose cur = follower.getPose();
        distanceToGoal = cur.distanceFrom(targett);// used to be getDistance();
        telemetry.addData("turret tick pos",turret.currentPos);
        telemetry.addData("align to tag",alignToTags);
        telemetry.addData("yaw to tag", turret.yawToTag);
        telemetry.addData("shooter sequence",shooterSequence);
        telemetry.addData("actual depo velo",depo.getVelocity());
        telemetry.addLine(shootingTest ? "Testing shooting using cross":"regular teleOp shooting");
        telemetry.addData("distance to goal",distanceToGoal);
        telemetry.addData("target velocity", ourVelo);
        telemetry.addData("shooting angle", LL.launchAngleServo.getPosition());
        telemetry.addData("X", cur.getX());
        telemetry.addData("y", cur.getY());
        telemetry.addData("heading", Math.toDegrees(cur.getHeading()));
        telemetry.addData("desired heading",Math.toDegrees(desiredHeading));
        if(depo.reachedTargetHighTolerance()){
            if (shooting) {
                // Decide which slot will actually shoot
//                if (LL.sensors.getLeft() != 0 && lastShotSlot != 0) {
//                    lastShotSlot = 0;
//                } else if (LL.sensors.getRight() != 0 && lastShotSlot != 1) {
//                    lastShotSlot = 1;
//                } else if (LL.sensors.getBack() != 0 && lastShotSlot != 2) {
//                    lastShotSlot = 2;
//                } else {
//                    // No valid new balls → cancel shooting
//                    depo.setTargetVelocity(0);
//                    shooting = false;
//                    lastShotSlot = -1;
//                    return;
//                }

                // Now start timer to shoot the 3 balls
                timer1.startTimer();
                shooting = false;
            }
            if(shooting2){
                timer2.startTimer();
                shooting2 = false;
            }
        }
        if(motif.equals("gpp")){
            if(greenInSlot == 0) LRBnoRecovery();
            else if(greenInSlot == 1) shootRBL();
            else shootBLR();
        }
        else if(motif.equals("pgp")){
            if(greenInSlot == 0) shootBLR();
            else if(greenInSlot == 1) shootLRB();
            else shootRBL();
        }
        else{
            if(greenInSlot == 0) shootRBL();
            else if(greenInSlot == 1) shootBLR();
            else shootLRB();
        }
//        shootoneColored();
//        shootMotifVelo(motif);
//        shootMotif(motif);
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
        if(dist<120){
        return (int) (3.69593*dist+960.60458);}//(3.69593*dist+929.60458) old
        else return 1650; //far
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
        if (dist>120) return 0.21;
        else return 0.00132566*dist+0.00291356;
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
    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setCameraPose(cameraPosition, cameraOrientation)
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        if (USE_WEBCAM) {
//            try { builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); }
//            catch (Exception e) { telemetry.addLine("Warning: Webcam not found"); }
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
    }
    private double getDistance(){ //old bot B logic
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
//        if (aprilTag == null) return;
//
//        List<AprilTagDetection> dets = aprilTag.getDetections();
//        tagDetected = false;
//
//        for (AprilTagDetection d : dets) {
//            if (d.metadata == null) continue;
//            if (d.metadata.name.contains("Obelisk")) continue;
//
//            tagDetected = true;
//
//            double xIn = d.robotPose.getPosition().x;
//            double yIn = d.robotPose.getPosition().y;
//            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//
//            ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
//
//            pedroPose = new Pose(ftcPose.getY() + 72, -ftcPose.getX() + 72, ftcPose.getHeading());
//            break;
//        }
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
    private void goToFarPose() {
        Pose target = bluealliance ? blueFarShootPose : redFarShootPose;
        Pose cur = follower.getPose();

        if (pedroPose == null) return;

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), target.getHeading())
                .build();

        follower.followPath(chain);
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
        Pose target = bluealliance ? blueGoal : redGoal;//faces the goal plus 10 inches on y to align better
        if (pedroPose == null) return;

        double rawAngle = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());

        double flippedAngle = rawAngle + Math.PI;
        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

        desiredHeading = flippedAngle;
        follower.turnTo(flippedAngle);
        aligning = true;
    }
    private void quitCorrectingAngle(){
        if(timer4.checkAtSeconds(1)){
            follower.breakFollowing();      // stops all paths and turns
            follower.startTeleopDrive();    // force drive mode back
            aligning = false;              // clear flags
            telemetry.addLine(">>> FORCED TELEOP CONTROL RESTORED <<<");
            timer4.stopTimer();
        }
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





    private boolean turnIsBasicallyDone() {
        Pose cur = follower.getPose();
        double error = Math.abs(desiredHeading - cur.getHeading());
        error = Math.abs((error + Math.PI) % (2 * Math.PI) - Math.PI);
        return error < Math.toRadians(2);  // 5° tolerance → good for unsticking small turns
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
    private void shootMotif(String seq){
        if(timer2.checkAtSeconds(0)) {//first shot
            if(seq.equals("gpp")) shootingHasWorkedNoVelo = LL.lift_green();
            else shootingHasWorkedNoVelo = LL.lift_purple();
            checkShotNoVelo();
        }
        if(timer2.checkAtSeconds(0.6)) {//second shot
            LL.allDown();
            if(seq.equals("pgp")) shootingHasWorkedNoVelo = LL.lift_green();
            else shootingHasWorkedNoVelo = LL.lift_purple();
            checkShotNoVelo();
        }
        if(timer2.checkAtSeconds(1.2)) {//third shot
            LL.allDown();
            if(seq.equals("ppg")) shootingHasWorkedNoVelo = LL.lift_green();
            else shootingHasWorkedNoVelo = LL.lift_purple();
            checkShotNoVelo();
        }
        if(timer2.checkAtSeconds(1.8)) {//tunr off depo
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
        }
    }
    private void shootMotifVelo(String seq){
        if (timer2.checkAtSeconds(0)&&shooterSequence==0) { //this executes when depo reached target so timer just started and we can fire the first shot
//            fireShotFromSlot(lastShotSlot); //lifts the first ball
//            rightFlip = true;
//            leftFlip = true;
//            backFlip = true;
//            if(seq.equals("gpp")) shootingHasWorked = LL.lift_green2(rightFlip,leftFlip,backFlip);
//            else shootingHasWorked = LL.lift_purple2(rightFlip,leftFlip,backFlip);
//            checkShot();
//            if(shootingHasWorked==1) rightFlip = false;//means right one worked first dont flip again later
//            else if(shootingHasWorked==0) leftFlip = false;
//            else if(shootingHasWorked==2) backFlip = false;
            boolean hasFired = false;
            if(seq.equals("gpp")){
                if(ballsInRobot[0]==1) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot1 = "Green from left";
                    hasFired = true;

                }
                else if(ballsInRobot[1]==1) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot1 = "Green from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==1) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot1 = "Green from back";
                    hasFired = true;
                }
            }
            else{
                if(ballsInRobot[0]==2) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot1 = "purp from left";
                    hasFired = true;
                }
                else if(ballsInRobot[1]==2) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot1 = "purp from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==2) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot1 = "purp from back";
                    hasFired = true;
                }
            }
            if(hasFired) {
                shooterSequence = 1; //this variable is a flag for the sequence to run properly
            }
            else{
                LL.allDown();
                depo.setTargetVelocity(0);
                timer2.stopTimer();
                shooterSequence = 0;
            }
        }

        // Shot 2
        if (timer2.checkAtSeconds(0.4)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
            LL.allDown();
            shooterSequence = 2;//sets up to check the depo velocity again
        }
        if(shooterSequence==2 && depo.reachedTargetHighTolerance()){ //this if statement is ran after depo reached target
//            fireNextAvailableShot();//lifts second ball
//            if(seq.equals("pgp")) shootingHasWorked = LL.lift_green2(rightFlip,leftFlip,backFlip);
//            else shootingHasWorked = LL.lift_purple2(rightFlip,leftFlip,backFlip);
//            checkShot();
//            if(shootingHasWorked==1) rightFlip = false;//means right one worked first dont flip again later
//            else if(shootingHasWorked==0) leftFlip = false;
//            else if(shootingHasWorked==2) backFlip = false;
            boolean hasFired = false;
            if(seq.equals("pgp")){
                if(ballsInRobot[0]==1) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot2 = "Green from left";
                    hasFired = true;

                }
                else if(ballsInRobot[1]==1) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot2 = "Green from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==1) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot2 = "Green from back";
                    hasFired = true;
                }
            }
            else{
                if(ballsInRobot[0]==2) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot2 = "purp from left";
                    hasFired = true;
                }
                else if(ballsInRobot[1]==2) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot2 = "purp from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==2) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot2 = "purp from back";
                    hasFired = true;
                }
            }
            if(hasFired) {
                shooterSequence=3;//sets the sequence to check
                timeOfSecondShot = timer2.timer.seconds()-timer2.curtime;//gets the curent time of the sequence so that next block runs now+0.4 instead of at a 0.8 seconds
            }
            else{
                LL.allDown();
                depo.setTargetVelocity(0);
                timer2.stopTimer();
                shooterSequence = 0;
            }

        }

        // Shot 3
        if (timer2.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
            LL.allDown();//puts the lifts down
            shooterSequence = 4;
        }
        if(shooterSequence==4 && depo.reachedTargetHighTolerance()){//does the velocity check again
//            if(seq.equals("ppg")) shootingHasWorked = LL.lift_green2(rightFlip,leftFlip,backFlip);
//            else shootingHasWorked = LL.lift_purple2(rightFlip,leftFlip,backFlip);
//            checkShot();
////            fireNextAvailableShot();
            boolean hasFired = false;
            if(seq.equals("ppg")){
                if(ballsInRobot[0]==1) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot3 = "Green from left";
                    hasFired = true;

                }
                else if(ballsInRobot[1]==1) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot3 = "Green from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==1) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot3 = "Green from back";
                    hasFired = true;
                }
            }
            else{
                if(ballsInRobot[0]==2) {
                    ballsInRobot[0]=0;
                    LL.leftUp();
                    shotSlot3 = "purp from left";
                    hasFired = true;
                }
                else if(ballsInRobot[1]==2) {
                    ballsInRobot[1]=0;
                    LL.rightUp();
                    shotSlot3 = "purp from right";
                    hasFired = true;
                }
                else if(ballsInRobot[2]==2) {
                    ballsInRobot[2]=0;
                    LL.backUp();
                    shotSlot3 = "purp from back";
                    hasFired = true;
                }
            }
            if(hasFired) {
                shooterSequence=5;
                timeOfSecondShot = timer2.timer.seconds()-timer2.curtime;
            }
            else{
                LL.allDown();
                depo.setTargetVelocity(0);
                timer2.stopTimer();
                shooterSequence = 0;
            }
            shooterSequence=5;
            timeOfSecondShot = timer2.timer.seconds()-timer2.curtime;
        }

        // Finish cycle
        if (timer2.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==5) {//resets the whole timer and sequence is done
            LL.allDown();
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            shooterSequence = 0;
        }
    }
    private void checkShot(){//checks that the correct color was shot otherwise quits shooting sequence
        if(shootingHasWorked==-1) {
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            LL.allDown();
            shooterSequence = 0;
        }
    }
    private void checkShotNoVelo(){//checks that the correct color was shot otherwise quits shooting sequence
        if(!shootingHasWorkedNoVelo) {
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            LL.allDown();
            shooterSequence = 0;
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

//hi