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
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_B_bot.lift_three;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Disabled
//
//hi..
@Autonomous(name = "closeRedMotif", group = "Pedro")
public class closeRedMotif extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 6, 12, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 180, -90, 0, 0);
    String motif = null;
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

    // Start at (0,0) with heading 20° to the RIGHT → -20° (clockwise negative)
    private final Pose startPose = new Pose(
            110,                // x inches
            -27,                     // y inches og:32
            Math.toRadians(-135)    // heading (rad)
    );

    // Your goal pose (exactly as in your movement program)
    private final Pose nearshotpose = new Pose(
            90,                    // x inches (forward) og: 72
            -8.5,                   // y inches (left)
            Math.toRadians(-220)    // heading (rad) at finish
    );
    private final Pose firstpickupPose = new Pose(
            66.5,                    // x inches (forward) og 71.5
            -6.5,                   // y inches (left) og: 22.5
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose secondpickupPose = new Pose(
            43.25,                    // x inches (forward) og 41.25
            -8,                   // y inches (left) og: 21
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose midpoint1     = new Pose(43.25,-2,  Math.toRadians(90.0));

    private final Pose thirdpickupPose = new Pose(
            20,                    // x inches (forward) og 16
            -9.5,                   // y inches (left) og: 22
            Math.toRadians(-90)    // heading (rad) at finish
    );
    private final Pose homePose = new Pose(
            0.0,                    // x inches (forward)
            0.0,            // y inches (left)
            Math.toRadians(-25.0)    // heading (rad) at finish
    );

    private final Pose infront_of_lever   = new Pose(61.5, -37.5, Math.toRadians(0));



    private static final double SECOND_HOP_IN = 20.0;
    private static final double SHOT_DELAY_S = 0.75;  // 🔹 delay between shots (tunable)

    // ---------- Timing for far shots ----------
    private Timer timer1;
    private Timer timer2;
    private int sequence = 0;
    boolean shootingHasWorkedNoVelo;

    int shooterSequence;


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
        depo    = new Deposition(hardwareMap);  // COMMENTED OUT (depo)
        LL      = new lift_three(hardwareMap);
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

        while (motif == null && !isStopRequested()) {
            InitialFindMotif();
            telemetry.addData("Looking for motif...", "");
            telemetry.update();
            idle();
            sleep(10);
        }
        if (motif.equals("pgp")) {
            motif = "ppg";
        }
        else if (motif.equals("ppg")){
            motif = "gpp";
        }
        else if (motif.equals("gpp")){
            motif = "pgp";
        }
        telemetry.addData("Motif Pattern:", motif);
        telemetry.update();

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


        telemetry.addLine("ADITI WAD HEREEEEEEE");
        telemetry.update();
        sleep(500);

    }
    private void go_infront(){
        Pose cur = follower.getPose();
        PathChain home = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(cur, infront_of_lever)))
                .setLinearHeadingInterpolation(cur.getHeading(), homePose.getHeading())
                .build();
        follower.followPath(home, true);
        while (opModeIsActive() && follower.isBusy()) { follower.update(); idle(); }
    }
    private void go_back(){

        Pose cur = follower.getPose();
        PathChain close_shot = follower.pathBuilder()
                .addPath(new Path(new BezierLine(cur, nearshotpose)))
                .setLinearHeadingInterpolation(cur.getHeading(), nearshotpose.getHeading())
                .setTimeoutConstraint(0.2)
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
    //ss
    private void three_far_shots() {
        LL.set_angle_far_auto();
        startFarShot();
        while (opModeIsActive() && !isFarShotCycleDone()) {
            depo.updatePID();  // COMMENTED OUT (depo)
            if (depo.reachedTarget()) {  // COMMENTED OUT (depo)
                if (sequence == 3 || sequence == 4) {
                    timer2.startTimer();
                    sequence = 0;
                }
            }
            shootMotif(motif);
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
                    timer2.startTimer();
                    sequence = 0;
                }
            }
            shootMotif(motif);
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
            idle();
        }


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
        double dy = (SECOND_HOP_IN+15) * Math.sin(heading);
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
            idle();
        }
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
        return (sequence == 0 && timer2.timerIsOff());
    }
    private void shoot3x(){
        if(timer1.checkAtSeconds(0)){
            LL.leftUp();
        }
        if(timer1.checkAtSeconds(0.3)){
            LL.leftDown();
            LL.rightUp();
        }
        if(timer1.checkAtSeconds(0.6)){
            LL.rightDown();
            LL.backUp();
        }
        if(timer1.checkAtSeconds(1.1)){
            LL.allDown();
            depo.setTargetVelocity(0);
            timer1.stopTimer();
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
    private void checkShotNoVelo(){//checks that the correct color was shot otherwise quits shooting sequence
        if(!shootingHasWorkedNoVelo) {
            depo.setTargetVelocity(0);
            timer2.stopTimer();
            LL.allDown();
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
                        motif = (detection.id == 21) ? "gpp" : (detection.id == 22) ? "pgp" : "ppg";
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