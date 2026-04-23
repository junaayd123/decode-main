package org.firstinspires.ftc.teamcode.pedroPathing.Misc;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "Misc Detect Only Red", group = "Misc")
public class IsolatedTurretDetectPickupRed extends OpMode {

    private Follower follower;
    private TurretLimelight turret;
    private VisionPortal visionPortal;
    private BallCoverageProcessor ballCoverage;

    private Servo camTilt = null;
    private DcMotor intake = null;

    private Timer pathTimeoutTimer;

    private PathChain scanPath;
    private PathChain collectPath;
    private PathChain returnPath;

    private int pathState = 0;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private double dynamicGateY = 30.0;

    // Matches the startPose in excess_farred_hough
    private static final Pose START_POSE  = new Pose(7 + 6.5, 7, Math.toRadians(0));
    // Shooting pose — this is where the auto scans from (detect in place after excess shot)
    private static final Pose SHOOT_POSE  = new Pose(12, 17, Math.toRadians(0));

    private static final double TURRET_DETECT_DEGREES = -8;

    // Gate collect X is fixed; Y is computed from regression
    private static final double GATE_COLLECT_X       = 74.0;
    private static final double GATE_COLLECT_HEADING = Math.toRadians(85);
    private static final double COLLECT_TIMEOUT_SEC  = 2.0;
    private static final double RETURN_TIMEOUT_SEC   = 2.0;

    // Sinusoidal regression: fieldY = A * sin(B * pixelX + C) + D
    private static final double REG_A = 25.0;
    private static final double REG_B = 0.00388657;
    private static final double REG_C = 2.20682;
    private static final double REG_D = 29.5;

    @Override
    public void init() {
        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        turret = new TurretLimelight(hardwareMap);
        turret.resetTurretEncoder();
        turret.setDegreesTarget(-63);

        intake   = hardwareMap.get(DcMotor.class, "intake");
        camTilt  = hardwareMap.get(Servo.class,   "cam_tilt");

        pathTimeoutTimer = new Timer();

        ballCoverage = new BallCoverageProcessor();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(ballCoverage);
        visionPortal = builder.build();
        // processor enabled by default — camera runs from init

        telemetry.addLine("A = drive to shoot pose | B = intake + collect + return");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
        aPressedLast = false;
        bPressedLast = false;
    }

    @Override
    public void loop() {
        follower.update();
        turret.toTargetInDegrees();

        boolean aPressed = gamepad1.a && !aPressedLast;
        aPressedLast = gamepad1.a;
        boolean bPressed = gamepad1.b && !bPressedLast;
        bPressedLast = gamepad1.b;

        switch (pathState) {
            case 0: // idle — wait for A button
                if (aPressed) {
                    turret.setDegreesTarget(TURRET_DETECT_DEGREES);
                    if (camTilt != null) camTilt.setPosition(0.1667);

                    Pose cur = follower.getPose();
                    scanPath = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(cur, SHOOT_POSE)))
                            .setLinearHeadingInterpolation(cur.getHeading(), SHOOT_POSE.getHeading())
                            .build();
                    follower.followPath(scanPath, false);
                    pathState = 1;
                }
                break;

            case 1: // at shoot pose, camera running — B to collect
                if (bPressed) {
                    // compute target Y from current hough reading
                    if (ballCoverage.houghCircleCount > 0 && ballCoverage.houghRawX >= 0) {
                        dynamicGateY = Math.max(7.0, REG_A * Math.sin(REG_B * ballCoverage.houghRawX + REG_C) + REG_D);
                    }
                    Pose cur = follower.getPose();
                    Pose collectTarget = new Pose(GATE_COLLECT_X, dynamicGateY, cur.getHeading());
                    collectPath = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(cur, collectTarget)))
                            .setConstantHeadingInterpolation(cur.getHeading())
                            .build();
                    intake.setPower(-1);
                    follower.followPath(collectPath, true);
                    pathTimeoutTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2: // driving to collect position
                intake.setPower(-1);
                if (!follower.isBusy() || pathTimeoutTimer.getElapsedTimeSeconds() >= COLLECT_TIMEOUT_SEC) {
                    Pose cur = follower.getPose();
                    returnPath = follower.pathBuilder()
                            .addPath(new Path(new BezierLine(cur, SHOOT_POSE)))
                            .setLinearHeadingInterpolation(cur.getHeading(), SHOOT_POSE.getHeading())
                            .build();
                    follower.followPath(returnPath, false);
                    pathTimeoutTimer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3: // returning to shoot pose
                if (!follower.isBusy() || pathTimeoutTimer.getElapsedTimeSeconds() >= RETURN_TIMEOUT_SEC) {
                    intake.setPower(0);
                    pathState = 1; // back to idle-at-shoot so B can be pressed again
                }
                break;
        }

        // === TELEMETRY ===
        String statusStr;
        if (pathState == 0) statusStr = "Idle — press A";
        else if (pathState == 1) statusStr = follower.isBusy() ? "Driving to shoot" : "Ready — press B";
        else if (pathState == 2) statusStr = "Collecting";
        else statusStr = "Returning";
        telemetry.addData("Status", statusStr);
        telemetry.addData("Pose X", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("Pose Y", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("Pose H deg", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Dynamic Gate Y", String.format("%.2f", dynamicGateY));
        telemetry.addLine("---");
        telemetry.addData("ROI coverage", String.format("%.2f%%", ballCoverage.roi1CombinedPercent));
        telemetry.addData("Circles detected", ballCoverage.houghCircleCount);
        telemetry.addData("Hough raw X", String.format("%.1f", ballCoverage.houghRawX));

        double[][] circles = ballCoverage.detectedCircles;
        if (circles != null && circles.length > 0) {
            for (int i = 0; i < circles.length; i++) {
                telemetry.addData("  Circle " + (i + 1),
                        String.format("x=%.0f  y=%.0f  r=%.0f px",
                                circles[i][0], circles[i][1], circles[i][2]));
            }
        } else {
            telemetry.addLine("  (no valid circles)");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intake != null) intake.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }

    static class BallCoverageProcessor implements VisionProcessor {
        private static final Scalar GREEN_LOWER = new Scalar(32, 50, 118);
        private static final Scalar GREEN_UPPER = new Scalar(255, 105, 145);
        private static final Scalar PURPLE_LOWER = new Scalar(32, 135, 135);
        private static final Scalar PURPLE_UPPER = new Scalar(255, 155, 169);

        private static final double ROTATE_DEGREES = -1.0; // net CCW

        // Single wide ROI spanning full frame, same height band
        private static final double ROI1_X_START = 0.0;
        private static final double ROI1_X_END   = 1.0;
        private static final double ROI1_Y_START  = 0.35;
        private static final double ROI1_Y_END    = 0.70;

        volatile double roi1CombinedPercent = 0;
        volatile double houghRawX = -1;
        volatile int houghCircleCount = 0;
        volatile double[][] detectedCircles = null; // each entry: {x, y, radius} in pixels
        volatile long frameSequence = 0;

        private final Mat rgbMat = new Mat();
        private final Mat rotatedMat = new Mat();
        private final Mat ycrcbMat = new Mat();
        private final Mat greenMask = new Mat();
        private final Mat purpleMask = new Mat();
        private final Mat combinedMat = new Mat();
        private final Mat roi1Mask = new Mat();
        private final Mat tempMask = new Mat();
        private final Mat vizMat = new Mat();
        private final Mat grayRoi = new Mat();
        private final Mat circlesMat = new Mat();
        private Bitmap vizBitmap;
        private Mat kernel;
        private Mat rotMatrix;
        private int frameWidth;
        private int frameHeight;

        private Point roi1TL, roi1BR;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            frameWidth = width;
            frameHeight = height;
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

            Point center = new Point(width / 2.0, height / 2.0);
            rotMatrix = Imgproc.getRotationMatrix2D(center, ROTATE_DEGREES, 1.0);

            roi1TL = new Point(width * ROI1_X_START, height * ROI1_Y_START);
            roi1BR = new Point(width * ROI1_X_END,   height * ROI1_Y_END);
            roi1Mask.create(height, width, CvType.CV_8UC1);
            roi1Mask.setTo(new Scalar(0));
            Imgproc.rectangle(roi1Mask, roi1TL, roi1BR, new Scalar(255), -1);

            vizBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, rgbMat, Imgproc.COLOR_RGBA2RGB);
            Imgproc.warpAffine(rgbMat, rotatedMat, rotMatrix,
                    new Size(frameWidth, frameHeight),
                    Imgproc.INTER_LINEAR, Core.BORDER_REPLICATE, new Scalar(0));
            Imgproc.cvtColor(rotatedMat, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

            Core.inRange(ycrcbMat, GREEN_LOWER, GREEN_UPPER, greenMask);
            Core.inRange(ycrcbMat, PURPLE_LOWER, PURPLE_UPPER, purpleMask);

            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.dilate(greenMask, greenMask, kernel, new Point(-1, -1), 2);
            Imgproc.erode(greenMask, greenMask, kernel, new Point(-1, -1), 1);

            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.dilate(purpleMask, purpleMask, kernel, new Point(-1, -1), 2);
            Imgproc.erode(purpleMask, purpleMask, kernel, new Point(-1, -1), 1);

            Core.bitwise_or(greenMask, purpleMask, combinedMat);

            double totalPixels = frameWidth * frameHeight;

            Core.bitwise_and(combinedMat, roi1Mask, tempMask);
            roi1CombinedPercent = Core.countNonZero(tempMask) / totalPixels * 100.0;

            Imgproc.cvtColor(rotatedMat, vizMat, Imgproc.COLOR_RGB2RGBA);
            Imgproc.rectangle(vizMat, roi1TL, roi1BR, new Scalar(255, 255, 0, 255), 2);

            Core.bitwise_and(combinedMat, roi1Mask, grayRoi);
            Mat houghKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7));
            Imgproc.dilate(grayRoi, grayRoi, houghKernel);
            Imgproc.erode(grayRoi, grayRoi, houghKernel);
            Imgproc.HoughCircles(grayRoi, circlesMat, Imgproc.HOUGH_GRADIENT,
                    1.5, 30, 50, 20, 10, 80);

            if (circlesMat.cols() > 0) {
                double maxX = -1;
                double[][] allCircles = new double[circlesMat.cols()][3];

                for (int i = 0; i < circlesMat.cols(); i++) {
                    double[] c = circlesMat.get(0, i);
                    Point ctr = new Point(c[0], c[1]);
                    int r = (int) Math.round(c[2]);

                    allCircles[i][0] = c[0];
                    allCircles[i][1] = c[1];
                    allCircles[i][2] = c[2];
                    if (c[0] > maxX) maxX = c[0];

                    Imgproc.circle(vizMat, ctr, r, new Scalar(0, 255, 255, 255), 2);
                    Imgproc.circle(vizMat, ctr, 3, new Scalar(255, 0, 0, 255), -1);
                    Imgproc.putText(vizMat,
                            String.format("(%d,%d) r=%d", (int) c[0], (int) c[1], r),
                            new Point(c[0] - 30, c[1] - r - 8),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.4,
                            new Scalar(255, 255, 255, 255), 1);
                }

                detectedCircles = allCircles;
                houghRawX = maxX;
                houghCircleCount = circlesMat.cols();
            } else {
                detectedCircles = null;
                houghRawX = -1;
                houghCircleCount = 0;
            }

            frameSequence++;
            Utils.matToBitmap(vizMat, vizBitmap);
            return vizBitmap;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasPxToImagePx,
                                Object userContext) {
            if (userContext instanceof Bitmap) {
                canvas.drawBitmap((Bitmap) userContext, null,
                        new RectF(0, 0, onscreenWidth, onscreenHeight), new Paint());
            }
        }
    }
}
