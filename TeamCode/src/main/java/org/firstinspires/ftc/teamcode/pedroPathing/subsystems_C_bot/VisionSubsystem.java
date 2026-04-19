package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import android.graphics.Canvas;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallProcessor ballProcessor;

    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9, 6, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);

    // BallProcessor constants
    static final double CAMERA_HEIGHT_M = 0.38;
    static final double CAMERA_PITCH_DEG = 0.0;
    static final double CAMERA_FOV_DEG = 78.0;
    static final double BALL_DIAMETER_M = 0.127;
    static final double ROTATION_DEG = -18.0;
    static final double ROI_TOP = 0.50;
    static final double CROP_BOTTOM = 0.20;
    static final Scalar GREEN_LOWER = new Scalar(32, 50, 118);
    static final Scalar GREEN_UPPER = new Scalar(255, 105, 145);
    static final Scalar PURPLE_LOWER = new Scalar(32, 135, 135);
    static final Scalar PURPLE_UPPER = new Scalar(255, 155, 169);
    static final double MIN_AREA = 10;
    static final double MAX_AREA = 100_000;
    static final double MIN_CIRCULARITY = 0.2;
    static final double EXPECTED_BALL_WIDTH_PX = 60;

    // Ramp Scan State
    private Timer rampScanTimer = new Timer();
    private boolean rampScanning = false;
    private final List<Integer> rampVotes = new ArrayList<>();
    private int rampBallVerdict = -1;

    public VisionSubsystem(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        ballProcessor = new BallProcessor();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
            }
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        builder.addProcessor(ballProcessor);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(true);

        visionPortal = builder.build();
        setBallProcessorEnabled(false);
    }

    public void update() {
        if (rampScanning) {
            rampVotes.add(ballProcessor.getTotalBalls());
            if (rampScanTimer.checkAtSeconds(1.0)) {
                finishRampScan();
            }
        }
    }

    public void startRampScan() {
        rampScanning = true;
        rampVotes.clear();
        rampBallVerdict = -1;
        setBallProcessorEnabled(true);
        rampScanTimer.startTimer();
    }

    private void finishRampScan() {
        rampScanning = false;
        setBallProcessorEnabled(false);
        if (!rampVotes.isEmpty()) {
            int[] freq = new int[10];
            for (int v : rampVotes) {
                if (v >= 0 && v < freq.length) freq[v]++;
            }
            int best = 0;
            for (int i = 1; i < freq.length; i++) {
                if (freq[i] > freq[best]) best = i;
            }
            rampBallVerdict = best;
        }
    }

    public boolean isRampScanning() {
        return rampScanning;
    }

    public int getRampBallVerdict() {
        return rampBallVerdict;
    }

    public int getCurrentBallCount() {
        return ballProcessor.getTotalBalls();
    }

    public int getRampVoteCount() {
        return rampVotes.size();
    }

    public List<BallDetection> getBallDetections() {
        return ballProcessor.getDetections();
    }

    public void setBallProcessorEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(ballProcessor, enabled);
    }

    public void setAprilTagEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTag.getDetections();
    }

    public Pose getLocalizedPose(double turretDeg) {
        List<AprilTagDetection> dets = getAprilTagDetections();
        for (AprilTagDetection d : dets) {
            if (d.metadata == null || d.metadata.name.contains("Obelisk")) continue;
            
            double xIn = d.robotPose.getPosition().x;
            double yIn = d.robotPose.getPosition().y;
            double hDeg = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            Pose ftcPose = new Pose(xIn, yIn, Math.toRadians(hDeg));
            return new Pose(
                    ftcPose.getY(),
                    -ftcPose.getX() + 72,
                    ftcPose.getHeading() + Math.toRadians(turretDeg));
        }
        return null;
    }

    public void close() {
        visionPortal.close();
    }

    public static class BallDetection {
        public String color;
        public int estimatedCount;
        public double fieldX, fieldY;
        public double circularity;
        public double ratioMultiple;
        public Rect boundingRect;
    }

    private static class BallProcessor implements VisionProcessor {
        private final Mat rotated = new Mat();
        private final Mat ycrcb = new Mat();
        private final Mat maskGreen = new Mat();
        private final Mat maskPurple = new Mat();
        private final Mat combined = new Mat();
        private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7, 7));
        private final List<BallDetection> detections = new ArrayList<>();
        private final Object lock = new Object();

        public List<BallDetection> getDetections() {
            synchronized (lock) { return new ArrayList<>(detections); }
        }

        public int getTotalBalls() {
            int total = 0;
            for (BallDetection d : getDetections()) total += d.estimatedCount;
            return total;
        }

        @Override public void init(int width, int height, CameraCalibration cal) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            int w = frame.cols(), h = frame.rows();
            Point center = new Point(w / 2.0, h / 2.0);
            Mat M = Imgproc.getRotationMatrix2D(center, -ROTATION_DEG, 1.0);
            Imgproc.warpAffine(frame, rotated, M, frame.size());
            M.release();

            int y1 = (int)(h * ROI_TOP);
            int y2 = (int)(h * (1.0 - CROP_BOTTOM));
            Rect roi = new Rect(0, y1, w, y2 - y1);
            Mat crop = rotated.submat(roi);

            Imgproc.cvtColor(crop, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(ycrcb, GREEN_LOWER, GREEN_UPPER, maskGreen);
            Core.inRange(ycrcb, PURPLE_LOWER, PURPLE_UPPER, maskPurple);

            for (Mat mask : new Mat[]{maskGreen, maskPurple}) {
                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
            }
            Core.bitwise_or(maskGreen, maskPurple, combined);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(combined, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<BallDetection> newDetections = new ArrayList<>();
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < MIN_AREA || area > MAX_AREA) continue;

                double peri = Imgproc.arcLength(new MatOfPoint2f(cnt.toArray()), true);
                double circ = (peri == 0) ? 0 : 4 * Math.PI * area / (peri * peri);
                if (circ < MIN_CIRCULARITY) continue;

                Rect localBbox = Imgproc.boundingRect(cnt);
                Rect bbox = new Rect(localBbox.x, localBbox.y + y1, localBbox.width, localBbox.height);

                double sumG = Core.sumElems(maskGreen.submat(localBbox)).val[0];
                double sumP = Core.sumElems(maskPurple.submat(localBbox)).val[0];
                String color = (sumG >= sumP) ? "GREEN" : "PURPLE";

                int count = Math.max(1, (int) Math.round((double) bbox.width / EXPECTED_BALL_WIDTH_PX));

                double cx = bbox.x + bbox.width / 2.0;
                double cy = bbox.y + bbox.height / 2.0;
                double rPx = (bbox.width / (double) count) / 2.0;
                double fX = w / (2.0 * Math.tan(Math.toRadians(CAMERA_FOV_DEG / 2.0)));
                double zCam = (fX * BALL_DIAMETER_M) / (2 * rPx);
                double xCam = (cx - w / 2.0) * zCam / fX;
                double yCam = (cy - h / 2.0) * zCam / fX;
                double pitch = Math.toRadians(CAMERA_PITCH_DEG);
                double xW = xCam;
                double yW = Math.cos(pitch) * yCam - Math.sin(pitch) * zCam;
                double zW = Math.sin(pitch) * yCam + Math.cos(pitch) * zCam;
                double t = CAMERA_HEIGHT_M / zW;

                BallDetection det = new BallDetection();
                det.color = color;
                det.estimatedCount = count;
                det.fieldX = xW * t;
                det.fieldY = yW * t;
                det.circularity = circ;
                det.ratioMultiple = (double) bbox.width / EXPECTED_BALL_WIDTH_PX;
                det.boundingRect = bbox;
                newDetections.add(det);
            }

            synchronized (lock) {
                detections.clear();
                detections.addAll(newDetections);
            }
            rotated.copyTo(frame);
            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    }
}
