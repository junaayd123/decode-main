package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.B_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@Autonomous(name = "far red final", group = "Pedro")
public class farRedfinal extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);
    String motif = "empty";
    String motifInit = "empty";
    private boolean shootingHasWorked = true;

    // ---------- Shooter subsystems -------------
    // Commented out: depo usage will be removed
    private Deposition depo;
    private lift_three LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower;
    boolean farparking = false;
    boolean shootallfar = false;
    private int linesToCollect = 3; // default to 3
    public ColorSensors sensors;


    // ----- Voltage-comp power (tune these once around ~12.35V) -----
    private static final double FAR_BASE_POWER_12V = 0.67;   // what worked for FAR at ~12.0V
    private static final double FAR_BASE_POWER2_12V = 0.675;  // a touch hotter between shots (optional)
    private static final double CLOSE_BASE_POWER_12V = 0.55;   // what worked for CLOSE at ~12.0V
    private static final double CLOSE_BASE_POWER2_12V = 0.55;  // what worked for CLOSE at ~12.0V

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose start_align_Pose = new Pose(-4.0, 2, Math.toRadians(-180));
    private final Pose startPose = new Pose(0.0, 0.0, Math.toRadians(-199));

    // Your goal pose (exactly as in your movement program)
    private final Pose firstpickupPose = new Pose(22.5, -20, Math.toRadians(-90));

    private final Pose midPoint1 = new Pose(37, -14, Math.toRadians(-90));
    private final Pose secondpickupPose = new Pose(45.5, -17, Math.toRadians(-90));

    private final Pose midPoint2 = new Pose(44, -4, Math.toRadians(-90));
    private final Pose thirdpickupPose = new Pose(71.5, -20, Math.toRadians(-90));
    private final Pose midPoint3 = new Pose(76, -4, Math.toRadians(-90));
    private final Pose near_shot_Pose = new Pose(97.5, -17, Math.toRadians(-237));
    private final Pose infront_of_lever = new Pose(61.5, -37.5, Math.toRadians(-180));
    private final Pose farPark = new Pose(
            25,
            -37,
            Math.toRadians(-180)
    );

    private static final double SECOND_HOP_IN = 13.5;
    boolean shootingHasWorkedNoVelo;
    private static final double SHOT_DELAY_S = 0.75;  // delay between shots (you already use timing windows below)

    // ---------- Upgraded shooter timing ----------
    private Timer timer1;
    private Timer timer2;
    private int sequence = 0;
    double timeOfSecondShot;
    int shooterSequence;
    int greenInSlot;


    // --------- Voltage-comp helpers (kept) ---------
    private double getBatteryVoltage() {
        double v = 0.0;
        for (com.qualcomm.robotcore.hardware.VoltageSensor vs : hardwareMap.voltageSensor) {
            double vv = vs.getVoltage();
            if (vv > 0) v = Math.max(v, vv);
        }
        return (v > 0) ? v : 12.35;
    }
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

    /**
     * Set both shooter motors with voltage compensation (base power is what you'd use at 12.0V).
     */
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
        depo = new Deposition(hardwareMap);  // COMMENTED OUT (depo)
        sensors = new ColorSensors(hardwareMap);
        LL = new lift_three(hardwareMap);
        timer1 = new Timer();
        timer2 = new Timer();

        intake = hardwareMap.get(DcMotor.class, "intake");  // COMMENTED OUT (intake)
        d1 = hardwareMap.get(DcMotor.class, "depo");   // ensure names match RC config
        d2 = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pedro follower (this is fine for Pedro 2.0)
        follower = B_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(start_align_Pose);

        initAprilTag();

        // Launcher safe start
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with depo PID + timer3) then run movement.");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            // D-pad Up -> 1 line, D-pad Left -> 2 lines, D-pad Right -> 3 lines
            if (gamepad1.dpadUpWasPressed()){
                if(linesToCollect ==3){
                    linesToCollect = 1;
                }
                else linesToCollect += 1;
            }
            if(gamepad1.crossWasPressed()){
                farparking = !farparking;
            }
            if(gamepad1.circleWasPressed()) shootallfar = !shootallfar;

            telemetry.addLine("Select # of lines using dpad up");
            telemetry.addLine("Select to park far using cross");
            telemetry.addLine("Select to shoot all far using circle");
            telemetry.addData("lines:", linesToCollect);
            if(farparking) telemetry.addLine("parking far");
            else telemetry.addLine("park at gate");
            if(shootallfar) telemetry.addLine("shoot all lines from far");
            else telemetry.addLine("last 2 lines will shoot close");
            if (motifInit.equals("empty")) InitialFindMotif();

            if (motifInit.equals("ppg")) {
                motif = "ppg";
                motifInit = "empty";
            } else if (motifInit.equals("pgp")) {
                motif = "pgp";
                motifInit = "empty";
            } else if (motifInit.equals("gpp")) {
                motif = "gpp";
                motifInit = "empty";
            }

            telemetry.addData("Motif", motif);
            telemetry.update();
            idle();
        }


        waitForStart();
        if (isStopRequested()) return;

        first_align_movement();
        three_far_shots();
        if (linesToCollect >= 1) {
            first_line_pickup();
            reset();
            go_home();
            three_far_shots();
        }
        if (linesToCollect >= 2) {
            second_line_pickup();
            reset();
            if(shootallfar){
                go_home();
                three_far_shots();
            }
            else {
                go_close();
                three_close_shots();
            }
        }
        if (linesToCollect >= 3) {
            third_line_pickup();
            reset();
            if(shootallfar){
                go_home();
                three_far_shots();
            }
            else {
                go_close_2();
                three_close_shots();
            }
        }

        reset();
        if(farparking) parkFar();
        else go_infront();

        telemetry.addLine("✅ Done: fired shots + completed paths.");
        telemetry.update();
        sleep(500);
    }
    private void parkFar(){
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, farPark)))
                .setLinearHeadingInterpolation(cur.getHeading(),farPark.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }

    }

    private void first_align_movement() {
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(start_align_Pose, startPose)))
                .setLinearHeadingInterpolation(start_align_Pose.getHeading(), startPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void go_infront() {
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur,infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
    }

    private void reset() {
        stopShooter();
        depo.setPowerBoth(0.0);              // COMMENTED OUT (depo)
    }

    //ss
    private void go_home() {
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, startPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), startPose.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)
    }

    // ===== Far / Close shot sequence starters (upgraded) =====
    private void startFarShot() {
        sequence = 3;
        depo.setTargetVelocity(depo.farVelo_New_auto);  // COMMENTED OUT (depo)
//        LL.far();
        // optionally: setShooterPowerVoltageComp(FAR_BASE_POWER_12V);
    }

    private void startCloseShot() {
        sequence = 4;
        depo.setTargetVelocity(depo.closeVelo_New);  // COMMENTED OUT (depo)
//        LL.close();

    }

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

    private void first_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        // path 1
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, firstpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        // path 2 (forward hop)
        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 18) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

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
        if (intake != null) intake.setPower(1);
    }

    private void second_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(startPose, midPoint1, secondpickupPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), secondpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur = follower.getPose();
        double heading = cur.getHeading();
        double dx = (SECOND_HOP_IN) * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 22) * Math.sin(heading);
        Pose secondGoal = new Pose(cur.getX() + dx, cur.getY() + dy, heading);

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
        if (intake != null) intake.setPower(1);
    }

    private void third_line_pickup() {
        if (intake != null) intake.setPower(-1);  // COMMENTED OUT (intake)
        Pose cur = follower.getPose();
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint3, thirdpickupPose)))
                .setLinearHeadingInterpolation(cur.getHeading(), thirdpickupPose.getHeading())
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }

        Pose cur1 = follower.getPose();
        double heading = cur1.getHeading();
        double dx = SECOND_HOP_IN * Math.cos(heading);
        double dy = (SECOND_HOP_IN + 13.5) * Math.sin(heading);
        Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);

        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur1, secondGoal)))
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
    private void checkShotNoVelo(){//checks that the correct color was shot otherwise quits shooting sequence
        if(!shootingHasWorkedNoVelo) {
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            LL.allDown();
            shooterSequence = 0;
        }
    }

    private void go_close() {

        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, midPoint2, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)

    }
    private void go_close_2() {
        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, near_shot_Pose)))
                .setLinearHeadingInterpolation(cur.getHeading(), near_shot_Pose.getHeading())
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
        if (intake != null) intake.setPower(0);  // COMMENTED OUT (intake)

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
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null &&
                        detection.metadata.name.contains("Obelisk")) {
                    motifInit = (detection.id == 21) ? "gpp"
                            : (detection.id == 22) ? "pgp" : "ppg";
                }
            }
        } catch (Exception ignored) {}
    }
}