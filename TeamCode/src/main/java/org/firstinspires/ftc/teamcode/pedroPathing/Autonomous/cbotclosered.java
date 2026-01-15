

package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.ColorSensors;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
//hi..
@Autonomous(name = "cbotclosered", group = "Pedro")
public class cbotclosered extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);
    private boolean shootingHasWorked = true;
    public ColorSensors sensors;

    // ---------- Shooter subsystems ----------
    private Deposition_C depo;
    private TurretLimelight turret;
    double shootinterval = 0.35;
    private lifters LL;
    private DcMotor intake = null;

    // Shooter motors (direct handles for voltage compensation)
    private DcMotor d1 = null;   // right shooter motor (example)
    private DcMotor d2 = null;   // left shooter motor  (example)

    // ---------- Pedro ----------
    private Follower follower;

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose startPose = new Pose(
            44,                // x inches
            128,                     // y inches og:32
            Math.toRadians(35)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose nearshotpose = new Pose(
            22,                    // x inches (forward) og: 72 //junaayd note: later change x and y so that they are exactly 12 instead of having this, meaning12 and 84
            90.5,                   // y inches (left)
            Math.toRadians(0)    // heading (rad) at finish
    );
    private final Pose nearshotpose2 = new Pose(//same pose but with heading of gate opening
            22,                    // x inches (forward) og: 72 //junaayd note: later change x and y so that they are exactly 12 instead of having this, meaning12 and 84
            90.5,                   // y inches (left)
            Math.toRadians(34)    // heading (rad) at finish
    );
    private final Pose firstPickupPose = new Pose(
            54,                    // x inches (forward) og: 72 //junaayd note: later change x and y so that they are exactly 12 instead of having this, meaning12 and 84
            84,                   // y inches (left)
            Math.toRadians(0)    // heading (rad) at finish
    );
    private final Pose midpoint1 = new Pose(
            13.4,                    // x inches (forward) og: 72
            58,                   // y inches (left)
            Math.toRadians(0)    // heading (rad) at finish
    );
    private final Pose midpoint2 = new Pose(
            10,                    // x inches (forward) og: 72
            68,                   // y inches (left)
            Math.toRadians(0)    // heading (rad) at finish
    );
    private final Pose firstpickupPose = new Pose(
            56,                    // x inches (forward) og 71.5
            55,                   // y inches (left) og: 22.5
            Math.toRadians(0)    // heading (rad) at finish
    );

    private final Pose midpointopengate = new Pose(
            13.4,
            68,
            Math.toRadians(0)
    );
    //private final Pose infront_of_lever   = new Pose(61.5, -36, Math.toRadians(180)); was modified 12/28
    private final Pose infront_of_lever   = new Pose(54, 60, Math.toRadians(0));
    private final Pose infront_of_lever_new  = new Pose(57.2, 57.1, Math.toRadians(34));
    private final Pose outfromgate = new Pose (50,50, Math.toRadians(42));

    private final Pose midpointbefore_intake_from_gate   = new Pose(52, 58, Math.toRadians(0));
    private final Pose intake_from_gate   = new Pose(56, 53, Math.toRadians(40));
    private final Pose intake_from_gate_rotate   = new Pose(55, 54, Math.toRadians(0));




    private static final double SECOND_HOP_IN = 20.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)

    // ---------- Timing for far shots ----------
    private Timer timer1;
    private Timer timer2;
    private Timer gateTimer;
    private int sequence = 0;
    boolean shootingHasWorkedNoVelo;

    double timeOfSecondShot;
    String motifInit = "empty";
    String motif = "empty";
    int shooterSequence;
    int greenInSlot;


    // ---------- Timing for far shots ----------



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
        depo    = new Deposition_C(hardwareMap);  // COMMENTED OUT (depo)
        LL      = new lifters(hardwareMap);
        sensors = new ColorSensors(hardwareMap);
        turret  = new TurretLimelight((hardwareMap));

        timer1  = new Timer();
        timer2  = new Timer();
        gateTimer = new Timer();

        intake  = hardwareMap.get(DcMotor.class, "intake");  // COMMENTED OUT (intake)
        d1      = hardwareMap.get(DcMotor.class, "depo");   // ensure names match RC config
        d2      = hardwareMap.get(DcMotor.class, "depo1");

        if (d1 != null) d1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (d2 != null) d2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pedro follower (this is fine for Pedro 2.0)
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        //    initAprilTag();

        // Launcher safe start
        LL.allDown();
        LL.set_angle_min();
        timer1.resetTimer();
        timer2.resetTimer();
        gateTimer.resetTimer();
        stopShooter();

        telemetry.addLine("Auto ready: will shoot 3 (far, with delay) then run your movement.");
        telemetry.update();

        turret.limelight.pipelineSwitch(2);
        turret.limelight.start();

        turret.resetTurretEncoder();
        turret.setDegreesTarget(-96.4);
        while (!isStarted() && !isStopRequested()) {
            turret.setPid();
            LLResult result = turret.limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults){
                if (fr.getFiducialId() == 21) {
                    motif= "pgp";
                }
                if (fr.getFiducialId() == 22) {
                    motif= "ppg";
                }
                if (fr.getFiducialId() == 23) {
                    motif=  "gpp";
                }
            }
            turret.toTargetInDegrees();

            telemetry.addData("Motif", motif);
            telemetry.update();
            idle();
        }
        waitForStart();
        if (isStopRequested()) return;
//        turret.setDegreesTarget(-25);
        turret.setDegreesTarget(-44.5);
        turret.setPid();

        go_back();
        three_close_shots();

        reset();
        turret.setDegreesTarget(22);
        bezier_curve_test(); // picks from 2nd line and comes back to near shooting zone

        three_close_shots();

        reset();
        go_gate_open(1.6); //opens gate and collects 3 artifacts and waits
//        go_back();

//        pauseBeforeShooting(.8);
        three_close_shots();

        reset();
        go_gate_open(1.2); //opens gate and collects 3 artifacts and waits
        //go_back();

//        pauseBeforeShooting(.8);
        three_close_shots();

        reset();
        go_gate_open(1.2); //opens gate and collects 3 artifacts and waits
        //go_back();

//        pauseBeforeShooting(.8);
        three_close_shots();


        //old auto function below
        //        first_line_pickup();
//        reset();
//        go_close();
//        three_close_shots();
//        second_line_pickup();
//        reset();
//        go_close();
//        three_close_shots();
//        third_line_pickup();
//        reset();
//        go_close();
//        three_close_shots();
//        go_infront();



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
    //hi
    private void go_back(){

        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                //.setBrakingStrength(1)
                .setTimeoutConstraint(0.2)
                .build();
        follower.followPath(close_shot, true);
        while (opModeIsActive() && follower.isBusy()) {
            turret.toTargetInDegrees();
            follower.update();
            manageSecondHopIntake();
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
            turret.toTargetInDegrees();
            idle();
        }
    }
    private void pauseWait(double seconds) {
        Timer pause = new Timer();
        pause.startTimer();
        while (opModeIsActive() && !pause.checkAtSeconds(seconds)) {
            follower.update();   // safe even if idle
            idle();
        }
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
            turret.toTargetInDegrees();
            if (depo.reachedTargetHighTolerance()) {  // COMMENTED OUT (depo)
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
    private void shootBLR(){//shoots in back left right order
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
    private void shootRBL() {//shoots in right back left order
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
    //ss


    private void bezier_curve_test() {
        intake.setPower(-1);
        Pose cur = follower.getPose();
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierCurve
                                        (cur,
                                                midpoint1,
                                                firstpickupPose)
                        )
                )
                .setLinearHeadingInterpolation(cur.getHeading(), firstpickupPose.getHeading(), 0.8)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        intake.setPower(0);
        cur = follower.getPose();
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain second = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierCurve
                                        (cur,
                                                midpoint2,
                                                nearshotpose2)
                        )
                )
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading(), 0.8)
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            manageSecondHopIntake();
            idle();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    private void open_gate() {
        //gateTimer.resetTimer();
        //intake.setPower(-1);
        Pose cur = follower.getPose();
        // ===== 2) Movement: your two-hop Pedro path =====
       /*PathChain first = follower.pathBuilder()
               .addPath(
                       new Path(
                               new BezierCurve
                                       (cur, infront_of_lever,midpointbefore_intake_from_gate, intake_from_gate)
                       )
               )
               .setLinearHeadingInterpolation(cur.getHeading(), intake_from_gate.getHeading(), 0.8)
               .build();*/
        PathChain first = follower.pathBuilder()
                .addPath(
                        new Path(new BezierLine(cur, infront_of_lever))
                )

                .setLinearHeadingInterpolation(cur.getHeading(), intake_from_gate.getHeading(), 0.8)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            //telemetry.addData("Timer Seconds", gateTimer.curtime);
            //telemetry.update();
            idle();

        }
        intake.setPower(-1);
        pauseWait(0.5);
        cur = follower.getPose();
        PathChain second = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierCurve(cur, midpointbefore_intake_from_gate, intake_from_gate, intake_from_gate_rotate)
                        )
                )
                .setLinearHeadingInterpolation(cur.getHeading(), intake_from_gate.getHeading(), intake_from_gate_rotate.getHeading())
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            //telemetry.addData("Timer Seconds", gateTimer.curtime);
            //telemetry.update();
            idle();

        }
        pauseWait(1.6);
        intake.setPower(0);

//       intake.setPower(0);
//       cur = follower.getPose();
//       // ===== 2) Movement: your two-hop Pedro path =====
//       PathChain second = follower.pathBuilder()
//               .addPath(
//                       new Path(
//                               new BezierCurve
//                                       (cur,
//                                               midpoint2,
//                                               nearshotpose)
//                       )
//               )
//               .setLinearHeadingInterpolation(cur.getHeading(), firstpickupPose.getHeading(), 0.8)
//               .build();
//       follower.followPath(second, true);
//       while (opModeIsActive() && follower.isBusy()) {
//           follower.update();
//           idle();
//       }
    }


    private void go_gate_open(double seconds){
        intake.setPower(-1);
        Pose cur = follower.getPose();
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, infront_of_lever_new)))
                .setLinearHeadingInterpolation(cur.getHeading(), infront_of_lever_new.getHeading(), 0.5)
                .setTimeoutConstraint(1.2)
                .build();
        follower.followPath(first, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            idle();
        }
        pauseWait(seconds);
        //
//        intake.setPower(0);
        cur = follower.getPose();
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain second = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, outfromgate, nearshotpose2)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose2.getHeading())
                .build();
        follower.followPath(second, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            manageSecondHopIntake();
            idle();
        }
    }
    private void first_line_pickup(){
        intake.setPower(-1);
        // ===== 2) Movement: your two-hop Pedro path =====
        PathChain first = follower.pathBuilder()
                .addPath(new Path(new BezierLine(nearshotpose, firstPickupPose)))
                .setLinearHeadingInterpolation(nearshotpose.getHeading(), firstPickupPose.getHeading(), 0.8)
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


    }
    /*private void second_line_pickup(){
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
        }*/

    // calculations to move forward
    /*Pose cur1 = follower.getPose();
    double heading = cur1.getHeading();
    double dx = (SECOND_HOP_IN) * Math.cos(heading);
    double dy = (SECOND_HOP_IN+15.5)* Math.sin(heading);
    Pose secondGoal = new Pose(cur1.getX() + dx, cur1.getY() + dy, heading);*/

    // second movement - 13 inch forward
    /*PathChain second = follower.pathBuilder()
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
        if (intake != null) intake.setPower(0);
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
        if (intake != null) intake.setPower(1);
    }
}

