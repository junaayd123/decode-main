//package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
//import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.regressions;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;
//
//import android.graphics.Canvas;
//
//import java.util.ArrayList;
//import java.util.List;
//
//@TeleOp(name = "Teleop solo", group = "A_TeleOp")
//public class BotCTeleop_solo extends OpMode {
//
//    // -----------------------------------------------------------------------
//    // OPENCV RAMP DETECTOR — ported from OpenCVRampVP
//    // -----------------------------------------------------------------------
//
//    // Camera + field parameters
//    static final double CAMERA_HEIGHT_M  = 0.38;
//    static final double CAMERA_PITCH_DEG = 0.0;
//    static final double CAMERA_FOV_DEG   = 78.0;
//    static final double BALL_DIAMETER_M  = 0.127;
//    static final double ROTATION_DEG     = -18.0;
//
//    // ROI crop fractions
//    static final double ROI_TOP     = 0.50;
//    static final double CROP_BOTTOM = 0.20;
//
//    // Color thresholds (YCbCr)
//    static final Scalar GREEN_LOWER  = new Scalar( 32,  50, 118);
//    static final Scalar GREEN_UPPER  = new Scalar(255, 105, 145);
//    static final Scalar PURPLE_LOWER = new Scalar( 32, 135, 135);
//    static final Scalar PURPLE_UPPER = new Scalar(255, 155, 169);
//
//    // Blob filters
//    static final double MIN_AREA               = 10;
//    static final double MAX_AREA               = 100_000;
//    static final double MIN_CIRCULARITY        = 0.2;
//    static final double EXPECTED_BALL_WIDTH_PX = 60;
//
//    /** Holds data about one detected blob on the ramp. */
//    static class BallDetection {
//        String color;
//        int    estimatedCount;
//        double fieldX, fieldY;
//        double circularity;
//        double ratioMultiple;
//        Rect   boundingRect;
//    }
//
//    /** VisionProcessor that runs on the shared VisionPortal alongside AprilTag. */
//    static class BallProcessor implements VisionProcessor {
//
//        // Red alliance only — rotation is always negative
//        final boolean isRedSide = true;
//
//        private final Mat rotated    = new Mat();
//        private final Mat ycrcb      = new Mat();
//        private final Mat maskGreen  = new Mat();
//        private final Mat maskPurple = new Mat();
//        private final Mat combined   = new Mat();
//        private final Mat kernel     = Imgproc.getStructuringElement(
//                Imgproc.MORPH_ELLIPSE, new Size(7, 7));
//
//        private final List<BallDetection> detections = new ArrayList<>();
//        private final Object lock = new Object();
//
//        public List<BallDetection> getDetections() {
//            synchronized (lock) { return new ArrayList<>(detections); }
//        }
//
//        /** Returns the total ball count across all blobs. */
//        public int getTotalBalls() {
//            int total = 0;
//            for (BallDetection d : getDetections()) total += d.estimatedCount;
//            return total;
//        }
//
//        @Override public void init(int width, int height, CameraCalibration cal) {}
//
//        @Override
//        public Object processFrame(Mat frame, long captureTimeNanos) {
//            int w = frame.cols(), h = frame.rows();
//
//            // Rotate to level the ramp (red side: positive ROTATION_DEG correction)
//            Point center = new Point(w / 2.0, h / 2.0);
//            double rotDeg = -ROTATION_DEG; // red side always
//            Mat M = Imgproc.getRotationMatrix2D(center, rotDeg, 1.0);
//            Imgproc.warpAffine(frame, rotated, M, frame.size());
//            M.release();
//
//            // Crop: keep the band between ROI_TOP and (1 - CROP_BOTTOM)
//            int y1   = (int)(h * ROI_TOP);
//            int y2   = (int)(h * (1.0 - CROP_BOTTOM));
//            Rect roi = new Rect(0, y1, w, y2 - y1);
//            Mat crop = rotated.submat(roi);
//
//            // Draw crop boundary on live view for tuning
//            Imgproc.rectangle(rotated,
//                    new Point(0, y1), new Point(w, y2),
//                    new Scalar(255, 255, 0), 2);
//
//            // Color thresholding (YCbCr)
//            Imgproc.cvtColor(crop, ycrcb, Imgproc.COLOR_RGB2YCrCb);
//            Core.inRange(ycrcb, GREEN_LOWER,  GREEN_UPPER,  maskGreen);
//            Core.inRange(ycrcb, PURPLE_LOWER, PURPLE_UPPER, maskPurple);
//
//            // Morphological cleanup
//            for (Mat mask : new Mat[]{maskGreen, maskPurple}) {
//                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
//                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
//                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
//                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
//            }
//            Core.bitwise_or(maskGreen, maskPurple, combined);
//
//            // Find contours
//            List<MatOfPoint> contours = new ArrayList<>();
//            Imgproc.findContours(combined, contours, new Mat(),
//                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            List<BallDetection> newDetections = new ArrayList<>();
//            for (MatOfPoint cnt : contours) {
//                double area = Imgproc.contourArea(cnt);
//                if (area < MIN_AREA || area > MAX_AREA) continue;
//
//                double peri = Imgproc.arcLength(new MatOfPoint2f(cnt.toArray()), true);
//                double circ = (peri == 0) ? 0 : 4 * Math.PI * area / (peri * peri);
//                if (circ < MIN_CIRCULARITY) continue;
//
//                // bbox: shift Y to full-frame coords for field-position math
//                Rect localBbox = Imgproc.boundingRect(cnt);
//                Rect bbox = new Rect(
//                        localBbox.x,
//                        localBbox.y + y1,
//                        localBbox.width,
//                        localBbox.height);
//
//                // Dominant color
//                double sumG = Core.sumElems(maskGreen .submat(localBbox)).val[0];
//                double sumP = Core.sumElems(maskPurple.submat(localBbox)).val[0];
//                String color = (sumG >= sumP) ? "GREEN" : "PURPLE";
//
//                int count = Math.max(1,
//                        (int) Math.round((double) bbox.width / EXPECTED_BALL_WIDTH_PX));
//
//                // Field position
//                double cx  = bbox.x + bbox.width  / 2.0;
//                double cy  = bbox.y + bbox.height / 2.0;
//                double rPx = (bbox.width / (double) count) / 2.0;
//
//                double fX   = w / (2.0 * Math.tan(Math.toRadians(CAMERA_FOV_DEG / 2.0)));
//                double zCam = (fX * BALL_DIAMETER_M) / (2 * rPx);
//                double xCam = (cx - w / 2.0) * zCam / fX;
//                double yCam = (cy - h / 2.0) * zCam / fX;
//
//                double pitch = Math.toRadians(CAMERA_PITCH_DEG);
//                double xW =  xCam;
//                double yW =  Math.cos(pitch) * yCam - Math.sin(pitch) * zCam;
//                double zW =  Math.sin(pitch) * yCam + Math.cos(pitch) * zCam;
//                double t  = CAMERA_HEIGHT_M / zW;
//
//                BallDetection det = new BallDetection();
//                det.color          = color;
//                det.estimatedCount = count;
//                det.fieldX         = xW * t;
//                det.fieldY         = yW * t;
//                det.circularity    = circ;
//                det.ratioMultiple  = (double) bbox.width / EXPECTED_BALL_WIDTH_PX;
//                det.boundingRect   = bbox;
//                newDetections.add(det);
//
//                // Draw on live view
//                Scalar drawColor = color.equals("GREEN")
//                        ? new Scalar(0, 255, 0)
//                        : new Scalar(128, 0, 128);
//                Imgproc.rectangle(rotated,
//                        new Point(bbox.x, bbox.y),
//                        new Point(bbox.x + bbox.width, bbox.y + bbox.height),
//                        drawColor, 2);
//                Imgproc.putText(rotated,
//                        color + " x" + count,
//                        new Point(bbox.x, Math.max(bbox.y - 5, 10)),
//                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 1);
//            }
//
//            synchronized (lock) {
//                detections.clear();
//                detections.addAll(newDetections);
//            }
//
//            rotated.copyTo(frame);
//            return frame;
//        }
//
//        @Override
//        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
//                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                                Object userContext) {}
//    }
//
//    // -----------------------------------------------------------------------
//    // ORIGINAL BotCTeleop FIELDS
//    // -----------------------------------------------------------------------
//
//    // Manual turret power applied when holding dpad left/right in Mode.nothing
//    static final double MANUAL_TURRET_POWER = 0.4;
//
//    private boolean aligning = false;
//    private boolean aligning2 = false;
//    private boolean alignForFar = false;
//    private double distanceToGoal;
//
//    private boolean bluealliance = false;
//    private boolean blueStartPose = false;
//    private double desiredHeading = 0;
//    String motif = "gpp";
//    String sequence = "rbl";
//
//    // --- RAMP CV state ---
//    private BallProcessor ballProcessor;
//    // ballOnRamp is now set from CV in addition to being set manually by button presses
//    int ballOnRamp;
//
//    public boolean tagInitializing;
//    int[] ballsInRobot = {0,0,0};
//    int greenInSlot;
//    private DcMotor intake = null;
//    private Deposition_C depo;
//    boolean shootingTest = false;
//    boolean intakeRunning;
//    int firstShot, secondShot, thirdShot;
//    Servo led;
//    Servo led2;
//    int lastShotSlot = -1;
//    private lifters LL;
//    boolean direction = false;
//    double speed;
//    Timer timer1;
//    Timer timersecondshot;
//    Timer timer3;
//    Timer timerthirdshot;
//    Timer timerfirstshot;
//    regressions reg;
//    double ourVelo = 1800;
//    boolean shooting = false;
//    double shootinterval = 0.35;
//    int shooterSequence;
//    int shooterSequenceFar;
//    double timeOfSecondShot;
//
//    Gamepad g1  = new Gamepad();
//    Gamepad preG1 = new Gamepad();
//    TurretLimelight turret;
//
//    private Follower follower;
//
//    private final Pose startPose    = new Pose(53, 70, 0);
//    private final Pose blueGoal     = new Pose(-72, 140, 0);
//    private final Pose redGoal      = new Pose(62, 137, 0);
//    private final Pose redGoalFixed = new Pose(72, 144, 0);
//    private final Pose blueGoalfar  = new Pose(-69, 144, 0);
//    private final Pose redGoalfar   = new Pose(62, 140, 0);
//    private final Pose rampPose = new Pose(72, 80, 0); // target pose turret faces to see ramp
//
//    // Camera setup
//    private static final boolean USE_WEBCAM = true;
//    private Position cameraPosition =
//            new Position(DistanceUnit.INCH, 0, 9, 6, 0);
//    private YawPitchRollAngles cameraOrientation =
//            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);
//
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//    Pose ftcPose, pedroPose;
//    boolean tagDetected;
//    double turretDeg;
//    Timer turretTimer;
//    double totalHedOffset;
//
//    private void initVision() {
//        // Build AprilTag processor
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
//                .setCameraPose(cameraPosition, cameraOrientation)
//                .build();
//
//        // Build BallProcessor (red side only)
//        ballProcessor = new BallProcessor();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            try { builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); }
//            catch (Exception e) { telemetry.addLine("Warning: Webcam not found"); }
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Add BOTH processors to the same portal
//        builder.addProcessor(aprilTag);
//        builder.addProcessor(ballProcessor);
//        builder.enableLiveView(true);
//        builder.setAutoStopLiveView(true);
//
//        visionPortal = builder.build();
//
//        // Start with ball detection paused until needed (saves CPU during normal play)
//        visionPortal.setProcessorEnabled(ballProcessor, false);
//    }
//
//    private void updateAprilTagLocalization() {
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
//            double xIn  = d.robotPose.getPosition().x;
//            double yIn  = d.robotPose.getPosition().y;
//            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
//
//            ftcPose  = new Pose(xIn, yIn, Math.toRadians(hDeg));
//            pedroPose = new Pose(
//                    ftcPose.getY(),
//                    -ftcPose.getX() + 72,
//                    ftcPose.getHeading() + Math.toRadians(turretDeg));
//            break;
//        }
//    }
//
//    private enum Mode { nothing, findTag, faceGoal, faceRamp }
//    private Mode mode = Mode.nothing;
//
//    // --- RAMP SCAN STATE ---
//    private Timer rampScanTimer;
//    private boolean rampScanning = false;
//    // Votes collected during the 1-second scan window (one entry per loop tick)
//    private final List<Integer> rampVotes = new ArrayList<>();
//    // The committed verdict set after scanning finishes
//    private int rampBallVerdict = -1; // -1 means not yet determined
//    private Mode modeBeforeRampScan = Mode.nothing; // mode to restore after scan
//
//    double headingTotag;
//    boolean flywheelEarlyStart;
//    private boolean frozen = false;
//    private Pose holdPose;
//
//    // -----------------------------------------------------------------------
//    // LIFECYCLE
//    // -----------------------------------------------------------------------
//
//    @Override
//    public void init() {
//        turret = new TurretLimelight(hardwareMap);
//        LL     = new lifters(hardwareMap);
//        depo   = new Deposition_C(hardwareMap);
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        follower = C_Bot_Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        g1.copy(gamepad1);
//        timer1          = new Timer();
//        timersecondshot = new Timer();
//        timer3          = new Timer();
//        timerthirdshot  = new Timer();
//        timerfirstshot  = new Timer();
//        turretTimer     = new Timer();
//        rampScanTimer   = new Timer();
//        reg = new regressions();
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        depo.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        depo.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        led  = hardwareMap.get(Servo.class, "led");
//        led2 = hardwareMap.get(Servo.class, "led2");
//
//        initVision(); // replaces the old initAprilTag()
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//        depo.setTargetVelocity(0);
//        LL.allDown();
//        LL.set_angle_min();
//        timer1.resetTimer();
//        timersecondshot.resetTimer();
//        timer3.resetTimer();
//        timerthirdshot.resetTimer();
//        timerfirstshot.resetTimer();
//        turretTimer.resetTimer();
//        rampScanTimer.resetTimer();
//        LL.set_camera_tag_pos();
//    }
//
//    // -----------------------------------------------------------------------
//    // MAIN LOOP
//    // -----------------------------------------------------------------------
//
//    @Override
//    public void loop() {
//        Pose cur = follower.getPose();
//        preG1.copy(g1);
//        g1.copy(gamepad1);
//        depo.updatePID();
//        follower.getTotalHeading();
//        turret.updateEncoderPos();
//
//        // --- RAMP SCAN TRIGGER: gamepad1.b starts a 1-second ramp scan ---
//        if (g1.b && !preG1.b && !rampScanning) {
//            LL.set_camera_ramp_pos();
//            startRampScan();
//        }
//
//        // --- LIVE RAMP BALL COUNT (only when actively scanning) ---
//        if (rampScanning && ballProcessor != null) {
//            rampVotes.add(ballProcessor.getTotalBalls());
//        }
//
//        // --- TURRET & GOAL GEOMETRY (unchanged) ---
//        Pose targett;
//        Pose targett2;
//        if (distanceToGoal > 125) {
//            targett = redGoalfar;
//        } else {
//            targett = redGoal;
//        }
//        targett2 = redGoalFixed;
//
//        double rawAngle = Math.atan2(
//                targett.getY() - cur.getY(),
//                targett.getX() - cur.getX());
//        double flippedAngle = rawAngle + Math.PI;
//        flippedAngle = ((flippedAngle + Math.PI) % (2 * Math.PI)) - Math.PI;
//        headingTotag = flippedAngle + Math.PI;
//
//        double robHeading = follower.getTotalHeading() - totalHedOffset;
//        while (robHeading >= Math.toRadians(210)) robHeading -= 2 * Math.PI;
//        while (robHeading < -Math.PI)              robHeading += 2 * Math.PI;
//
//        // --- INTAKE (unchanged) ---
//        if (g1.rightBumperWasPressed()) {
//            LL.allDown();
//            if (intake.getPower() < -0.5) {
//                intake.setPower(0);
//                intakeRunning = false;
//            } else {
//                intake.setPower(-1);
//                intakeRunning = true;
//            }
//        }
//        if (shooting) {
//            intake.setPower(0);
//            intakeRunning = false;
//        }
//        if (intakeRunning) {
//            led.setPosition(0.28);
//            led2.setPosition(0.28);
//            if (LL.sensors.getRight() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getLeft() != 0) {
//                timer3.startTimer();
//                intakeRunning = false;
//                led.setPosition(0.5);
//                led2.setPosition(0.5);
//            }
//        }
//        if (g1.left_bumper) {
//            intake.setPower(1);
//        } else if (!g1.left_bumper && !intakeRunning && !timer3.timerIsOn()) {
//            intake.setPower(0);
//        }
//
//        reverseIntake();
//
//        if (g1.cross) speed = 0.3;
//        else          speed = 1;
//
//        if (g1.psWasPressed()) {
//            if      (motif.equals("gpp")) motif = "pgp";
//            else if (motif.equals("pgp")) motif = "ppg";
//            else                          motif = "gpp";
//        }
//
//        // --- TURRET MODES ---
//        if (mode == Mode.faceGoal) {
//            turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingTotag));
//        }
//        if (mode == Mode.nothing) {
//            // Manual turret control: hold dpad_left to pan left, dpad_right to pan right
//            if (g1.dpad_left) {
//                turret.toTargetInDegrees2(Math.toDegrees(turret.currentPos / 670.0 * Math.PI) - 1.5);
//            } else if (g1.dpad_right) {
//                turret.toTargetInDegrees2(Math.toDegrees(turret.currentPos / 670.0 * Math.PI) + 1.5);
//            }
//        }
//        if (mode == Mode.findTag) {
//            turret.toTargetInDegrees();
//        }
//        if (mode == Mode.faceRamp) {
//            // Same geometry as faceGoal, but we must normalize the result to [-180, 180]
//            // so the turret always takes the shortest path. Without normalization the
//            // formula can produce values like -281 deg (equivalent to +79 deg) which
//            // converts to ticks past the hardware limit, jamming the turret to one side.
//            double rawRampAngle = Math.atan2(rampPose.getY() - cur.getY(), rampPose.getX() - cur.getX());
//            double flippedRamp  = rawRampAngle + Math.PI;
//            flippedRamp = ((flippedRamp + Math.PI) % (2 * Math.PI)) - Math.PI;
//            double headingToRamp = flippedRamp + Math.PI;
//            double rampDeg = Math.toDegrees(robHeading - headingToRamp);
//            // Normalize to [-180, 180] — pick the shortest arc
//            while (rampDeg >  180) rampDeg -= 360;
//            while (rampDeg < -180) rampDeg += 360;
//            turret.toTargetInDegrees2(rampDeg);
//        }
//
//        // --- RAMP SCAN COMPLETION CHECK ---
//        if (rampScanning && rampScanTimer.checkAtSeconds(0.3)) {
//            rampVotes.clear();
//        }
//        if (rampScanTimer.checkAtSeconds(1) && rampScanning) {
//            LL.allDown();
//            finishRampScan();
//            ballOnRamp = rampBallVerdict % 3;
//            greenInSlot = getGreenPos();
//        }
//        if (rampScanTimer.checkAtSeconds(1.3)) {
//            rampScanTimer.stopTimer();
//            depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
//            LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
//            shooting = true;
//        }
//
//        if (g1.triangleWasPressed()) {
//            if (tagInitializing) {
//                tagInitializing = false;
//                led.setPosition(0);
//                mode = Mode.nothing;
//                pauseAprilTagDetection();
//            } else {
//                LL.set_camera_tag_pos();
//                turretTimer.startTimer();
//                mode = Mode.findTag;
//                turret.setDegreesTarget(0);
//                led.setPosition(0.34);
//                resumeAprilTagDetection();
//            }
//        }
//        if (turretTimer.checkAtSeconds(1)) {
//            tagInitializing = true;
//            turretTimer.stopTimer();
//        }
//        if (tagInitializing) {
//            updateAprilTagLocalization();
//            if (tagDetected && pedroPose != null) {
//                mode = Mode.faceGoal;
//                telemetry.addLine("seeing and localizing tag");
//                telemetry.addData("local x",   pedroPose.getX());
//                telemetry.addData("local y",   pedroPose.getY());
//                telemetry.addData("local hed", Math.toDegrees(pedroPose.getHeading()));
//                telemetry.addData("turret angle", turretDeg);
//                follower.setPose(pedroPose.getPose());
//                totalHedOffset = follower.getTotalHeading() - pedroPose.getHeading();
//                tagInitializing = false;
//                LL.set_camera_ramp_pos();
//                led.setPosition(0.6);
//                pauseAprilTagDetection();
//            }
//        }
//        if(gamepad1.psWasPressed()){
//            shootingTest=!shootingTest;
//        }
//
//
//        // --- SHOOT INTERVAL BASED ON DISTANCE (unchanged) ---
//        if      (distanceToGoal > 125) shootinterval = 0.2;
//        else                           shootinterval = 0.17;
//
//        // --- SHOOTING TRIGGERS (unchanged, but ballOnRamp already pre-filled by CV) ---
//        if (g1.squareWasPressed()) { // shoot 3 close
//            LL.allDown();
//            if (!LL.checkNoBalls()) {
//                if (shootingTest) {
//                    depo.setTargetVelocity(ourVelo);
//                } else {
//                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
//                    LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
//                }
//                ballOnRamp = 0;
//                greenInSlot = getGreenPos();
//                shooting = true;
//            }
//        }
//        if (!shootingTest) {
//            if (shooting) depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
//            LL.set_angle_custom(angleBasedOnDistance(distanceToGoal));
//            if (flywheelEarlyStart) {
//                if (LL.sensors.getLeft() != 0 && LL.sensors.getBack() != 0 && LL.sensors.getRight() != 0) {
//                    depo.setTargetVelocity(veloBasedOnDistance(distanceToGoal));
//                } else if (!shooting && timer1.timerIsOff()) {
//                    depo.setTargetVelocity(0);
//                }
//            }
//        }
//        if(g1.leftStickButtonWasPressed()) {
//            ourVelo-=50;
//        }if(g1.rightStickButtonWasPressed()) {
//            ourVelo+=50;
//        }
//        if(g1.dpadUpWasPressed()){
//            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()+0.01);
//        }if(g1.dpadDownWasPressed()){
//            LL.launchAngleServo.setPosition(LL.launchAngleServo.getPosition()-0.01);
//        }
//        if (g1.shareWasPressed()) {
//            timer1.stopTimer();
//            intakeRunning = false;
//            shooting = false;
//            intake.setPower(0);
//            depo.setTargetVelocity(0);
//            LL.allDown();
//        }
//
//        followerstuff();
//
//        // --- DISTANCE + SHOOTER SEQUENCE (unchanged) ---
//        distanceToGoal = cur.distanceFrom(targett2);
//
//        if (depo.reachedTargetHighTolerance()) {
//            if (shooting) {
//                timer1.startTimer();
//                shooting = false;
//            }
//        }
//
//        // --- MOTIF → SEQUENCE LOGIC (unchanged) ---
//        if (motif.equals("gpp")) {
//            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) sequence = "lrb";
//            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) sequence = "rbl";
//            else sequence = "blr";
//        } else if (motif.equals("pgp")) {
//            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) sequence = "blr";
//            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) sequence = "lrb";
//            else sequence = "rbl";
//        } else { // ppg
//            if      ((ballOnRamp==0&&greenInSlot==0)||(ballOnRamp==1&&greenInSlot==2)||(ballOnRamp==2&&greenInSlot==1)) sequence = "rbl";
//            else if ((ballOnRamp==0&&greenInSlot==1)||(ballOnRamp==1&&greenInSlot==0)||(ballOnRamp==2&&greenInSlot==2)) sequence = "blr";
//            else sequence = "lrb";
//        }
//
//        if      (sequence.equals("lrb")) LRBnoRecovery();
//        else if (sequence.equals("rbl")) RBLnoRecovery();
//        else                             BLRnoRecovery();
//
//        // --- TELEMETRY ---
//        telemetry.addData("motif",            motif);
//        telemetry.addData("turret tick pos",  turret.currentPos);
//        telemetry.addData("shooter sequence", shooterSequence);
//        telemetry.addData("first shot velo",  firstShot);
//        telemetry.addData("second shot",      secondShot);
//        telemetry.addData("third shot",       thirdShot);
//        telemetry.addLine(shootingTest ? "Testing shooting using cross" : "regular teleOp shooting");
//        telemetry.addData("distance to goal", distanceToGoal);
//        telemetry.addData("actual depo velo", depo.getVelocity());
//        telemetry.addData("target velocity",  shootingTest ? ourVelo : depo.targetVelocity);
//        telemetry.addData("shooting angle",   LL.launchAngleServo.getPosition());
//        telemetry.addData("X",                cur.getX());
//        telemetry.addData("y",                cur.getY());
//        telemetry.addData("heading",          Math.toDegrees(cur.getHeading()));
//        telemetry.addData("total heading",    Math.toDegrees(follower.getTotalHeading() - totalHedOffset));
//        telemetry.addData("green in slot",    greenInSlot);
//        telemetry.addData("turret mode",      mode);
//        // Ramp CV telemetry
//        telemetry.addLine("--- RAMP CV ---");
//        telemetry.addData("Scanning (g1.b to trigger)", rampScanning);
//        telemetry.addData("Ramp ball verdict", rampBallVerdict < 0 ? "not yet scanned" : String.valueOf(rampBallVerdict));
//        if (rampScanning) {
//            telemetry.addData("Votes collected so far", rampVotes.size());
//            telemetry.addData("Current raw count", ballProcessor != null ? ballProcessor.getTotalBalls() : 0);
//        }
//
//        telemetry.update();
//    }
//
//    // -----------------------------------------------------------------------
//    // HELPERS (all unchanged from original)
//    // -----------------------------------------------------------------------
//
//    private int veloBasedOnDistance(double dist) {
//        return reg.distanceToVelo(dist);
//    }
//    private double angleBasedOnDistance(double dist) {
//        return reg.distanceToAngle(dist);
//    }
//
//    private void reverseIntake() {
//        if (timer3.checkAtSeconds(0))   intake.setPower(1);
//        if (timer3.checkAtSeconds(0.5)) {
//            intake.setPower(0);
//            timer3.stopTimer();
//        }
//    }
//
//    private void followerstuff() {
//        follower.update();
//        if (!follower.isBusy() && !aligning) {
//            if (direction) {
//                follower.setTeleOpDrive(
//                        gamepad1.left_stick_y * speed,
//                        (gamepad1.right_trigger - gamepad1.left_trigger) * speed,
//                        -gamepad1.right_stick_x * speed, true);
//            } else {
//                follower.setTeleOpDrive(
//                        -gamepad1.left_stick_y * speed,
//                        (gamepad1.left_trigger - gamepad1.right_trigger) * speed,
//                        -gamepad1.right_stick_x * speed, true);
//            }
//        }
//    }
//
//    private int getGreenPos() {
//        int pos = LL.sensors.getLeft();
//        if (pos == 1) return 0;
//        pos = LL.sensors.getRight();
//        return (pos == 1) ? 2 : 1;
//    }
//
//    private void LRBnoRecovery() {
//        if (timer1.checkAtSeconds(0))                       { firstShot = (int) depo.getVelocity(); LL.leftUp();  shooterSequence = 1; }
//        if (timer1.checkAtSeconds(shootinterval)          && shooterSequence==1) { secondShot=(int)depo.getVelocity(); LL.allDown(); LL.rightUp(); shooterSequence=2; }
//        if (timer1.checkAtSeconds(shootinterval*2)        && shooterSequence==2) { thirdShot =(int)depo.getVelocity(); LL.allDown(); LL.backUp();  shooterSequence=3; }
//        if (timer1.checkAtSeconds(shootinterval*3+0.25)   && shooterSequence==3) { LL.allDown(); depo.setTargetVelocity(0); timer1.stopTimer(); shooterSequence=0; }
//    }
//
//    private void BLRnoRecovery() {
//        if (timer1.checkAtSeconds(0))                            { LL.backUp();  shooterSequence=1; }
//        if (timer1.checkAtSeconds(shootinterval)   && shooterSequence==1) { LL.allDown(); LL.leftUp();  shooterSequence=2; }
//        if (timer1.checkAtSeconds(shootinterval*2) && shooterSequence==2) { LL.allDown(); LL.rightUp(); shooterSequence=3; }
//        if (timer1.checkAtSeconds(shootinterval*3+0.25) && shooterSequence==3) { LL.allDown(); depo.setTargetVelocity(0); timer1.stopTimer(); shooterSequence=0; }
//    }
//
//    private void RBLnoRecovery() {
//        if (timer1.checkAtSeconds(0))                            { LL.rightUp(); shooterSequence=1; }
//        if (timer1.checkAtSeconds(shootinterval)   && shooterSequence==1) { LL.allDown(); LL.backUp();  shooterSequence=2; }
//        if (timer1.checkAtSeconds(shootinterval*2) && shooterSequence==2) { LL.allDown(); LL.leftUp();  shooterSequence=3; }
//        if (timer1.checkAtSeconds(shootinterval*3+0.25) && shooterSequence==3) { LL.allDown(); depo.setTargetVelocity(0); timer1.stopTimer(); shooterSequence=0; }
//    }
//
//    // -----------------------------------------------------------------------
//    // RAMP SCAN HELPERS
//    // -----------------------------------------------------------------------
//
//    /**
//     * Kicks off a 1-second ramp scan:
//     *  - enables the ball processor
//     *  - points the turret at rampPose using the same faceGoal geometry
//     *  - clears the vote buffer and starts the scan timer
//     */
//    private void startRampScan() {
//        modeBeforeRampScan = mode;          // remember what we were doing
//        mode = Mode.faceRamp;
//        rampScanning = true;
//        rampVotes.clear();
//        visionPortal.setProcessorEnabled(ballProcessor, true);
//        rampScanTimer.startTimer();
//    }
//
//    /**
//     * Called once the 1-second window expires.
//     * Tallies votes by majority and commits the verdict to {@code rampBallVerdict}.
//     * Restores the previous turret mode and disables the ball processor.
//     */
//    private void finishRampScan() {
//        rampScanning = false;
//        visionPortal.setProcessorEnabled(ballProcessor, false);
//
//        // Majority-vote across all ticks collected during the scan window
//        if (!rampVotes.isEmpty()) {
//            // Count frequency of each result (0–9 balls is plenty)
//            int[] freq = new int[10];
//            for (int v : rampVotes) {
//                if (v >= 0 && v < freq.length) freq[v]++;
//            }
//            int best = 0;
//            for (int i = 1; i < freq.length; i++) {
//                if (freq[i] > freq[best]) best = i;
//            }
//            rampBallVerdict = best;
//        }
//
//        // Restore whatever turret mode we had before (faceGoal, nothing, etc.)
//        mode = modeBeforeRampScan;
//    }
//
//    private void pauseAprilTagDetection() {
//        if (visionPortal != null && aprilTag != null)
//            visionPortal.setProcessorEnabled(aprilTag, false);
//    }
//
//    private void resumeAprilTagDetection() {
//        if (visionPortal != null && aprilTag != null)
//            visionPortal.setProcessorEnabled(aprilTag, true);
//    }
//}