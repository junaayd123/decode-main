package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Deposition;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


//hi..
@Autonomous(name = "close blue final", group = "Pedro")
public class closeBluefinal extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);
    String motif = "empty";
    String motifInit = "empty";
    private boolean shootingHasWorked = true;

    // ---------- Shooter subsystems ----------
    private Deposition depo;
    private lift_three LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower;
    double timeOfSecondShot;
    public ColorSensors sensors;
    int greenInSlot;//0 if in left 1 if right, 2 if back
    private final Pose startPose = new Pose(
            107.313,               // x inches
            34.687,                // y inches
            Math.toRadians(135)
    );

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose nearshotpose     = new Pose(93.5,  12, Math.toRadians(-124));
    private final Pose firstpickupPose  = new Pose(66.5, 13,  Math.toRadians(90));
    private final Pose secondpickupPose = new Pose(43.25,12,  Math.toRadians(90));
    private final Pose midpoint1        = new Pose(43.25,4,  Math.toRadians(90.0));
    private final Pose thirdpickupPose  = new Pose(18,   14,  Math.toRadians(90));
    private final Pose homePose         = new Pose(0.0,  0.0, Math.toRadians(16.2));
    private final Pose infront_of_lever   = new Pose(59.5, 36.5, Math.toRadians(180));
    //

    boolean shootingHasWorkedNoVelo;
    private static final double SECOND_HOP_IN = 19.75;
    private static final double SHOT_DELAY_S  = 0.75;
    int shooterSequence;


    // ---------- Timing for far shots ----------
    private Timer timer1;
    private Timer timer2;
    private int sequence = 0;


    private double getBatteryVoltage() {
        double v = 0.0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor vs : hardwareMap.voltageSensor) {
            double vv = vs.getVoltage();
            if (vv > 0) v = Math.max(v, vv);
        }
        return (v > 0) ? v : 12.35;
    }

    /** Set both shooter motors with voltage compensation (base power is what you'd use at 12.0V). */
    private void setShooterPowerVoltageComp(double basePowerAt12V) {
        double v = getBatteryVoltage();          // e.g., 13.2V fresh, 12.0V nominal, 11.5V lower
        double compensated = basePowerAt12V * (12.35 / v);
        compensated = Math.max(0.0, Math.min(1.0, compensated));

        if (d1 != null) d1.setPower(compensated);
        if (d2 != null) d2.setPower(compensated);
        telemetry.addData("ShooterVComp", "V=%.2f base=%.3f out=%.3f", v, basePowerAt12V, compensated);
    }

    private void stopShooter() {
        if (d1 != null) d1.setPower(0.0);
        if (d2 != null) d2.setPower(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Init subsystems
        depo    = new Deposition(hardwareMap);  // COMMENTED OUT (depo)
        LL      = new lift_three(hardwareMap);
        sensors = new ColorSensors(hardwareMap);

        timer1  = new Timer();
        timer2  = new Timer();

        intake  = hardwareMap.get(DcMotor.class, "intake");  // COMMENTED OUT (intake)
        d1      = hardwareMap.get(DcMotor.class, "depo");   // ensure names match RC config
        d2      = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pedro follower (this is fine for Pedro 2.0)
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        initAprilTag();

        // Launcher safe start
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay) then run your movement.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if(motifInit.equals("empty")) InitialFindMotif();

            if (motifInit.equals("ppg")) {
                motif = "pgp";
                motifInit="empty";
            }
            else if (motifInit.equals("pgp")){
                motif = "gpp";
                motifInit="empty";
            }
            else if (motifInit.equals("gpp")){
                motif = "ppg";
                motifInit="empty";
            }
            telemetry.addData("Motif Pattern:", motif);
            telemetry.addData("Looking for motif...", "");
            telemetry.update();
            idle();
            sleep(10);
        }


        waitForStart();
        if (isStopRequested()) return;



        go_back();
        pauseBeforeShooting(0.4);
        three_close_shots();
        first_line_pickup();
        reset();
        go_close();
        three_close_shots();
        second_line_pickup();
        reset();
        go_close();
        three_close_shots();
        third_line_pickup();
        reset();
        go_close();
        three_close_shots();
        go_infront();

        telemetry.update();
        sleep(500);

    }
    private void go_infront(){
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        if (intake != null) intake.setPower(0);

    }
    private void go_back(){

        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);              // COMMENTED OUT (depo)

    }


    // ===== Far shot logic (exact from your teleOp) =====
    private void startFarShot() {
        sequence = 3;
        depo.setTargetVelocity(depo.farVelo_New);  // COMMENTED OUT (depo)
//        LL.far();
        // optionally: setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
    }

    private void startCloseShot() {
        sequence = 4;
        depo.setTargetVelocity(depo.closeVelo_New);  // COMMENTED OUT (depo)
//        LL.close();

    }
    private void pauseBeforeShooting(double seconds) {
        Timer pause = new Timer();
        pause.startTimer();
        while (opModeIsActive() && !pause.checkAtSeconds(seconds)) {
            follower.update();   // safe even if idle
            idle();
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
    //ss
    private void three_far_shots() {
        LL.set_angle_far_auto();
        startFarShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();  // COMMENTED OUT (depo)
            if (depo.reachedTarget()) {  // COMMENTED OUT (depo)
                if (sequence == 3 || sequence == 4) {
                    greenInSlot = getGreenPos();
                    timer1.startTimer();
                    sequence = 0;
                }
            }
            if(motif.equals("gpp")){
                if(greenInSlot == 0) shootLRB();
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
            follower.update();
        }
    }

    private void three_close_shots() {
        LL.set_angle_close();
        startCloseShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();  // COMMENTED OUT (depo)
            if (depo.reachedTarget()) {  // COMMENTED OUT (depo)
                if (sequence == 3 || sequence == 4) {
                    greenInSlot = getGreenPos();
                    timer1.startTimer();
                    sequence = 0;
                }
            }
            if(motif.equals("gpp")){
                if(greenInSlot == 0) shootLRB();
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
            follower.update();
        }
    }


    private void first_line_pickup(){
        intake.setPower(-1);
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(nearshotpose, firstpickupPose)))
                .setLinearHeadingInterpolation(nearshotpose.getHeading(), firstpickupPose.getHeading(), 0.8)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+7.5) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);
        Path p2 = new Path(new BezierLine(cur, secondGoal));

        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(p2)
                .setConstantHeadingInterpolation(heading)
                .setTimeoutConstraint(0.2)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // 👉 PROTECTION AGAINST 4TH/5TH BALL DURING SECOND HOP
            manageSecondHopIntake();


            idle();
        }
        if (intake != null) intake.setPower(1);

    }
    private void second_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        intake.setPower(-1);
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(nearshotpose,secondpickupPose))) //add the midpoint
                .setLinearHeadingInterpolation(nearshotpose.getHeading(),secondpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+17) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

        follower.setMaxPower(0.7);
        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .setTimeoutConstraint(0.2)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // 👉 PROTECTION AGAINST 4TH/5TH BALL DURING SECOND HOP
            manageSecondHopIntake();


            idle();
        }
        follower.setMaxPower(1);
        if (intake != null) intake.setPower(1);

    }
    private void third_line_pickup(){
        // ===== 2) Movement: your two-hop Pedro path =====
        intake.setPower(-1);
        Pose cur = follower.getPose();
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, thirdpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(),thirdpickupPose.getHeading())
                .setTimeoutConstraint(0.2)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

        // calculations to move forward
        Pose cur1 = follower.getPose();
        double heading = cur1.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN+15.5)* Math.sin(heading);
        Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);

        // second movement - 13 inch forward
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur1, secondGoal)))
                .setConstantHeadingInterpolation(heading)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // 👉 PROTECTION AGAINST 4TH/5TH BALL DURING SECOND HOP
            manageSecondHopIntake();


            idle();
        }
        if (intake != null) intake.setPower(1);
    }
    /**
     * Manages intake during the second hop.
     * If robot already has 1–2 balls in storage, spit out any new ones.
     * If robot already has 3, completely stop intake.
     * If robot has 0, intake normally.
     */
    private void manageSecondHopIntake() {
        if (intake == null || LL == null || sensors == null) return;

        boolean rightFull = (sensors.getRight() != 0);
        boolean backFull  = (sensors.getBack()  != 0);
        boolean leftFull  = (sensors.getLeft()  != 0);

        int count = 0;
        if (rightFull) count++;
        if (backFull)  count++;
        if (leftFull)  count++;

        // Tray full → spit everything else we touch
        if (count >= 3) {
            if (intake != null) intake.setPower(1);
            return;
        }

        // Tray not full yet → continue grabbing balls
        if (intake != null) intake.setPower(-1);
    }

    private void go_close(){
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midpoint1,nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(),nearshotpose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        intake.setPower(0);
    }

    private boolean isFarShotCycleDone() {
        return (sequence == 0 && timer1.timerIsOff());
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

    private void shootLRB() {//shoots in left right back order
//        if (lastShotSlot == -1) return; // nothing scheduled

        if (timer1.checkAtSeconds(0)) { //this executes when depo reached target so timer just started and we can fire the first shot
//            fireShotFromSlot(lastShotSlot); //lifts the first ball
            LL.leftUp();
            shooterSequence = 1; //this variable is a flag for the sequence to run properly
        }

        // Shot 2
        if (timer1.checkAtSeconds(0.4)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
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
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
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
        if (timer1.checkAtSeconds(0.4)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
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
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
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
        if (timer1.checkAtSeconds(0.4)&&shooterSequence==1) {//after 0.4 sec after first shot starts puts the lifts down
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
        if (timer1.checkAtSeconds(0.4+timeOfSecondShot)&&shooterSequence==3) {//at 0.4 seconds after 2nd lift
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
        }
    }

    private void checkShot() {
        if (!shootingHasWorked) {
            LL.allDown();
            shootingHasWorked = true;
        }
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
                telemetry.addLine("Warning: Webcam not found");
            }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void initOpenCV() {
        // LEAVE BLANK
    }
    private void InitialFindMotif() {
        try {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    // Only use tags that don't have Obelisk in them
                    if (detection.metadata.name.contains("Obelisk")) {
                        motifInit = (detection.id == 21) ? "gpp" : (detection.id == 22) ? "pgp" : "ppg";
                        telemetry.addData("motif: ", motif);
                    }   // end for() loop

                    // Add "key" information to telemetry
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

                }
            }
        } catch (Exception e) {
            telemetry.addData("Exception:", e);
        }
    }
}