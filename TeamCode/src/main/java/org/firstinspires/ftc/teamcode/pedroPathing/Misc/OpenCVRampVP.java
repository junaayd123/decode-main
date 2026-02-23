package org.firstinspires.ftc.teamcode.pedroPathing.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import android.graphics.Canvas;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "VISIONPORTAL Ball Detection", group = "Vision")
public class OpenCVRampVP extends LinearOpMode {

    // note to shyam and misha, i wrote this in python and used Claude to convert to VISIONPORTAL code
    // there will be some non standard stuff so you can delete if needed

    // -----------------------------------------------------------------------
    // CAMERA + FIELD PARAMETERS
    // -----------------------------------------------------------------------
    static final double CAMERA_HEIGHT_M  = 0.38;
    static final double CAMERA_PITCH_DEG = 0.0;
    static final double CAMERA_FOV_DEG   = 78.0;
    static final double BALL_DIAMETER_M  = 0.127;
    static final double ROTATION_DEG     = -18.0;

    //adding the ROI stuff
    static final double ROI_TOP    = 0.50;  // skip this fraction from the top
    static final double CROP_BOTTOM = 0.20;  // skip this fraction from the bottom

    //NOTE TO SHYAM AND MISHA: THESE VALUES ARE IN YCBCR (luminance brightness, blue difference, red difference)
    // THIS IS NOT HSV OR RGB
    static final Scalar GREEN_LOWER  = new Scalar( 32,  50, 118);
    static final Scalar GREEN_UPPER  = new Scalar(255, 105, 145);
    static final Scalar PURPLE_LOWER = new Scalar( 32, 135, 135);
    static final Scalar PURPLE_UPPER = new Scalar(255, 155, 169);

    // -----------------------------------------------------------------------
    // FILTERING
    // -----------------------------------------------------------------------
    static final double MIN_AREA           = 10;
    static final double MAX_AREA           = 100_000;
    static final double MIN_CIRCULARITY    = 0.2;

    // so i did this thing a while ago where i took expected RADIUS and then i did division
    //to calculate the number of balls in one blob
    //i took this same idea and then used it with width in pixels
    static final double EXPECTED_BALL_WIDTH_PX = 60;

    // -----------------------------------------------------------------------
    // DETECTION RESULT
    // -----------------------------------------------------------------------
    static class BallDetection {
        String color;
        int    estimatedCount;   // balls in this blob
        double fieldX, fieldY;
        double circularity;
        double aspectRatio;      // w/h of the bounding box (debug)
        double ratioMultiple;    // aspectRatio / EXPECTED_RATIO (debug)
        Rect   boundingRect;
    }

    // -----------------------------------------------------------------------
    // VISION PROCESSOR
    // -----------------------------------------------------------------------
    static class BallProcessor implements VisionProcessor {

        volatile boolean isRedSide = false;

        private final Mat rotated    = new Mat();
        private final Mat ycrcb      = new Mat();
        private final Mat maskGreen  = new Mat();
        private final Mat maskPurple = new Mat();
        private final Mat combined   = new Mat();
        private final Mat kernel     = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE, new Size(7, 7));

        private final List<BallDetection> detections = new ArrayList<>();
        private final Object lock = new Object();

        public List<BallDetection> getDetections() {
            synchronized (lock) { return new ArrayList<>(detections); }
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        //note to shyam and misha: some of this processing stuff was just leftover in the python file so just ignore it
        //remove it with claude or gpt if you need to, just keep the actual ratio processing, rotation, and other stuff in there
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            int w = frame.cols(), h = frame.rows();

            // 1. Rotate to level the ramp
            Point center = new Point(w / 2.0, h / 2.0);
            // 1. Rotate to level the ramp — flip sign based on alliance side
            double rotDeg = isRedSide ? -ROTATION_DEG : ROTATION_DEG;
            Mat M = Imgproc.getRotationMatrix2D(center, rotDeg, 1.0);
            Imgproc.warpAffine(frame, rotated, M, frame.size());
            M.release();

            // 2. Crop: keep only the band between ROI_TOP and (1 - CROP_BOTTOM)
            int y1   = (int)(h * ROI_TOP);
            int y2   = (int)(h * (1.0 - CROP_BOTTOM));
            Rect roi = new Rect(0, y1, w, y2 - y1);
            Mat crop = rotated.submat(roi);  // zero-copy window into rotated

            // Draw the crop boundary on the live view so you can tune it on the DS
            Imgproc.rectangle(rotated,
                    new Point(0, y1), new Point(w, y2),
                    new Scalar(255, 255, 0), 2);

            // 3. Color thresholding on the cropped region only
            Imgproc.cvtColor(crop, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(ycrcb, GREEN_LOWER,  GREEN_UPPER,  maskGreen);
            Core.inRange(ycrcb, PURPLE_LOWER, PURPLE_UPPER, maskPurple);

            // 4. Morphological cleanup
            for (Mat mask : new Mat[]{maskGreen, maskPurple}) {
                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.dilate(mask, mask, kernel, new Point(-1,-1), 2);
                Imgproc.erode (mask, mask, kernel, new Point(-1,-1), 2);
            }

            Core.bitwise_or(maskGreen, maskPurple, combined);

            // 5. Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(combined, contours, new Mat(),
                    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            List<BallDetection> newDetections = new ArrayList<>();
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < MIN_AREA || area > MAX_AREA) continue;

                double peri = Imgproc.arcLength(new MatOfPoint2f(cnt.toArray()), true);
                double circ = (peri == 0) ? 0 : 4 * Math.PI * area / (peri * peri);
                if (circ < MIN_CIRCULARITY) continue;

                // bbox is crop-local — shift Y up to full-frame coords for drawing + math
                Rect localBbox = Imgproc.boundingRect(cnt);
                Rect bbox = new Rect(
                        localBbox.x,
                        localBbox.y + y1,
                        localBbox.width,
                        localBbox.height);

                // Dominant color (use local coords for the mask lookup)
                double sumG = Core.sumElems(maskGreen .submat(localBbox)).val[0];
                double sumP = Core.sumElems(maskPurple.submat(localBbox)).val[0];
                String color = (sumG >= sumP) ? "GREEN" : "PURPLE";

                int count = Math.max(1,
                        (int) Math.round((double) bbox.width / EXPECTED_BALL_WIDTH_PX));

                // Field position uses full-frame coords so the math stays correct
                double cx  = bbox.x + bbox.width  / 2.0;
                double cy  = bbox.y + bbox.height / 2.0;
                double rPx = (bbox.width / (double) count) / 2.0;

                double fX   = w / (2.0 * Math.tan(Math.toRadians(CAMERA_FOV_DEG / 2.0)));
                double zCam = (fX * BALL_DIAMETER_M) / (2 * rPx);
                double xCam = (cx - w / 2.0) * zCam / fX;
                double yCam = (cy - h / 2.0) * zCam / fX;

                double pitch = Math.toRadians(CAMERA_PITCH_DEG);
                double xW =  xCam;
                double yW =  Math.cos(pitch) * yCam - Math.sin(pitch) * zCam;
                double zW =  Math.sin(pitch) * yCam + Math.cos(pitch) * zCam;
                double t  = CAMERA_HEIGHT_M / zW;

                double aspectRatio   = (double) bbox.width / bbox.height;
                double ratioMultiple = (double) bbox.width / EXPECTED_BALL_WIDTH_PX;

                BallDetection det = new BallDetection();
                det.color          = color;
                det.estimatedCount = count;
                det.fieldX         = xW * t;
                det.fieldY         = yW * t;
                det.circularity    = circ;
                det.aspectRatio    = aspectRatio;
                det.ratioMultiple  = ratioMultiple;
                det.boundingRect   = bbox;
                newDetections.add(det);

                // Draw on the full rotated frame (full-frame coords)
                Scalar drawColor = color.equals("GREEN")
                        ? new Scalar(0, 255, 0)
                        : new Scalar(128, 0, 128);
                Imgproc.rectangle(rotated,
                        new Point(bbox.x, bbox.y),
                        new Point(bbox.x + bbox.width, bbox.y + bbox.height),
                        drawColor, 2);
                Imgproc.putText(rotated,
                        color + " x" + count,
                        new Point(bbox.x, Math.max(bbox.y - 5, 10)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 1);
            }
            //adding the synchronized thing so that only a single thread runs this
            synchronized (lock) {
                detections.clear();
                detections.addAll(newDetections);
            }

            // now this is showing post-processing frame and then giving it to the display
            rotated.copyTo(frame);

            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {}
    }

    // opmode for the testing
    @Override
    public void runOpMode() {
        BallProcessor processor = new BallProcessor();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        boolean prevA = false;

        while (opModeIsActive()) {
            // Toggle alliance side on gamepad1.a press
            boolean currA = gamepad1.a;
            if (currA && !prevA) {
                processor.isRedSide = !processor.isRedSide;
            }
            prevA = currA;
            List<BallDetection> detections = processor.getDetections();

            // Sum up total balls across all blobs
            int totalBalls = 0;
            for (BallDetection d : detections) totalBalls += d.estimatedCount;

            telemetry.addData("Side (A to toggle)", processor.isRedSide ? "RED (+18°)" : "BLUE (-18°)");
            telemetry.addData("Total balls on ramp", totalBalls);
            telemetry.addData("Blobs detected",      detections.size());
            telemetry.addLine("---");
            for (int i = 0; i < detections.size(); i++) {
                BallDetection d = detections.get(i);
                telemetry.addLine(String.format(
                        "[%d] %s x%d  X=%.2fm Y=%.2fm  circ=%.2f  w=%dpx (%.2fx expected)",
                        i, d.color, d.estimatedCount,
                        d.fieldX, d.fieldY,
                        d.circularity, d.boundingRect.width, d.ratioMultiple));
            }
            telemetry.update();

            sleep(20);
        }

        portal.close();
    }
}