package org.firstinspires.ftc.teamcode.pedroPathing;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TunablePurpleBallPipeline extends OpenCvPipeline {

    // ---------- Camera / ball ----------
    public static int IMG_W = 640;
    public static int IMG_H = 480;
    public static double HFOV_DEG = 60.0;
    public static double VFOV_DEG = fovVFromH(HFOV_DEG, IMG_W, IMG_H); // derived each frame

    public static double BALL_DIAM_IN = 5.0;     // physical diameter
    public static double BALL_RADIUS_IN = 2.5;   // center height above floor for a ball on the floor

    // --- Camera mounting (for ground-plane distance) ---
    public static double CAMERA_HEIGHT_IN = 9.25;
    public static double CAMERA_PITCH_DEG = 18.5;   // you said 10° down
    public static double Y_DEG_MIN = 1;

    // ---------- HSV (OpenCV ranges: H 0..179, S/V 0..255) ----------
    // Good purple starting point; tune in Dashboard:
    // H: 130..160, S: 80..255, V: 50..255
    public static int H_LO = 130, S_LO = 70, V_LO = 90;
    public static int H_HI = 175, S_HI = 255, V_HI = 255; // narrow H a bit to reject false positives
    public static boolean SHOW_MASK = false;

    // ---------- Pre / post ----------
    public static int BLUR_SIZE = 5;
    // Make OPEN smaller so we don't erase a large, near ball
    public static int OPEN_SIZE = 10;    // was 18
    public static int CLOSE_SIZE = 5;
    public static boolean USE_HOUGH = true;

    // ---------- HoughCircles ----------
// Up-close ball can be ~230 px diameter => radius ~115 px.
// Let’s give lots of headroom.
    public static double HC_DP = 1.2;
    public static double HC_MIN_DIST = 30;
    public static double HC_PARAM1 = 160;  // a bit gentler than 180
    public static double HC_PARAM2 = 22;   // slightly more sensitive
    public static int HC_MIN_RADIUS = 8;
    public static int HC_MAX_RADIUS = 220;   // was 80 → allow large near-ball

    // ---------- EMA smoothing of diameter ----------
    public static boolean USE_EMA = true;
    public static double DIAM_ALPHA = 0.3;

    // ---------- Output ----------
    public static class Result {
        public boolean hasTarget = false;
        public int cx=0, cy=0;
        public double radiusPx=0, diameterPx=0;

        // distanceIn: pinhole (diameter-based, camera-height assumption)
        public double distanceIn = Double.NaN;

        // NEW: ground-plane distance using vertical bearing + camera height/pitch
        public double groundDistanceIn = Double.NaN;

        public double bearingXDeg=Double.NaN; // +right
        public double bearingYDeg=Double.NaN; // +up
    }

    private final Object lock = new Object();
    private final Result latest = new Result();

    // Reusable Mats
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat work = new Mat();
    private final Mat morph = new Mat();
    private final Mat grayBgr = new Mat();
    private final Mat gray = new Mat();
    private final Mat circles = new Mat();
    private final Mat hierarchy = new Mat();

    // EMA state
    private double emaDiameter = Double.NaN;

    @Override
    public Mat processFrame(Mat input) {
        // Update derived VFOV in case user tweaks IMG_W/H/HFOV live
        VFOV_DEG = fovVFromH(HFOV_DEG, IMG_W, IMG_H);

        // BGR -> HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // HSV threshold
        Scalar lo = new Scalar(clamp(H_LO,0,179), clamp(S_LO,0,255), clamp(V_LO,0,255));
        Scalar hi = new Scalar(clamp(H_HI,0,179), clamp(S_HI,0,255), clamp(V_HI,0,255));
        Core.inRange(hsv, lo, hi, mask);

        // Blur
        int k = makeOdd(BLUR_SIZE);
        if (k >= 3) Imgproc.GaussianBlur(mask, work, new Size(k,k), 0);
        else         mask.copyTo(work);

        // Morph open + close
        Mat kOpen  = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(makeOdd(OPEN_SIZE), makeOdd(OPEN_SIZE)));
        Mat kClose = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(makeOdd(CLOSE_SIZE), makeOdd(CLOSE_SIZE)));
        Imgproc.morphologyEx(work, morph, Imgproc.MORPH_OPEN, kOpen);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, kClose);

        // Prepare grayscale for Hough
        Imgproc.cvtColor(morph, grayBgr, Imgproc.COLOR_GRAY2BGR);
        Imgproc.cvtColor(grayBgr, gray, Imgproc.COLOR_BGR2GRAY);

        boolean found = false;
        int cx=0, cy=0; double radius=0;

        if (USE_HOUGH) {
            circles.release();
            Imgproc.HoughCircles(
                    gray, circles, Imgproc.HOUGH_GRADIENT,
                    HC_DP, HC_MIN_DIST, HC_PARAM1, HC_PARAM2,
                    HC_MIN_RADIUS, HC_MAX_RADIUS
            );
            if (circles.cols() > 0) {
                double[] c = circles.get(0,0);
                if (c != null && c.length >= 3) {
                    cx = (int)Math.round(c[0]);
                    cy = (int)Math.round(c[1]);
                    radius = c[2];
                    found = radius > 1;
                }
            }
        }

        if (!found) {
            // Fallback: largest contour -> minEnclosingCircle
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double bestArea = 0;
            MatOfPoint best = null;
            for (MatOfPoint c : contours) {
                double a = Imgproc.contourArea(c);
                if (a > bestArea) { bestArea = a; best = c; }
            }
            if (best != null) {
                Point ctr = new Point();
                float[] r = new float[1];
                Imgproc.minEnclosingCircle(new MatOfPoint2f(best.toArray()), ctr, r);
                cx = (int)Math.round(ctr.x);
                cy = (int)Math.round(ctr.y);
                radius = r[0];
                found = radius > 1;
            }
        }

        if (found) {
            int diameterPxRaw = (int)Math.round(2.0 * radius);

            // Smooth diameter if enabled
            double diaForDist;
            if (USE_EMA) {
                emaDiameter = Double.isNaN(emaDiameter) ? diameterPxRaw
                        : (DIAM_ALPHA * diameterPxRaw + (1.0 - DIAM_ALPHA) * emaDiameter);
                diaForDist = emaDiameter;
            } else {
                diaForDist = diameterPxRaw;
            }

            // Pinhole distance (inches) based on diameter
            double focalPx = focalLengthPx(HFOV_DEG, IMG_W);
            double distanceIn = (diaForDist <= 1e-6) ? Double.POSITIVE_INFINITY
                    : (focalPx * BALL_DIAM_IN / diaForDist);

            // Bearings (deg): +right, +up
            double pxPerDegX = IMG_W / HFOV_DEG;
            double pxPerDegY = IMG_H / VFOV_DEG;
            double errX = cx - (IMG_W/2.0);
            double errY = (IMG_H/2.0) - cy;
            double bearingXDeg = errX / pxPerDegX;
            double bearingYDeg = errY / pxPerDegY;

            // ---- NEW: ground-plane distance using vertical bearing ----
            // define downward angle (alpha): camera pitch (+down) plus -bearingY (down is positive)
            double alphaDeg = CAMERA_PITCH_DEG + (-bearingYDeg);
            double alphaRad = Math.toRadians(alphaDeg);
            double groundDistanceIn = Double.POSITIVE_INFINITY;
            double hToCenter = CAMERA_HEIGHT_IN - BALL_RADIUS_IN; // distance from camera height to ball center height

            if (Math.abs(alphaDeg) >= Y_DEG_MIN && hToCenter > 0) {
                double tan = Math.tan(alphaRad);
                if (Math.abs(tan) > 1e-6) {
                    groundDistanceIn = hToCenter / tan;
                    if (groundDistanceIn < 0) groundDistanceIn = Double.POSITIVE_INFINITY; // behind camera: reject
                }
            }

            // Draw circle + center
            Imgproc.circle(input, new Point(cx, cy), (int)Math.round(radius), new Scalar(0,255,0), 2);
            Imgproc.circle(input, new Point(cx, cy), 3, new Scalar(0,0,255), -1);
            Imgproc.putText(input, String.format("Dia: %dpx  Dist(pinhole): %.2fin", diameterPxRaw, distanceIn),
                    new Point(Math.max(5, cx-60), Math.max(18, cy-16)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255), 1);
            Imgproc.putText(input, String.format("Bx: %.1f deg  Ground: %.2fin", bearingXDeg, groundDistanceIn),
                    new Point(Math.max(5, cx-60), Math.max(36, cy+16)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255), 1);

            synchronized (lock) {
                latest.hasTarget = true;
                latest.cx = cx; latest.cy = cy;
                latest.radiusPx = radius;
                latest.diameterPx = 2.0 * radius;
                latest.distanceIn = distanceIn;
                latest.groundDistanceIn = groundDistanceIn;   // <-- NEW
                latest.bearingXDeg = bearingXDeg;
                latest.bearingYDeg = bearingYDeg;
            }
        } else {
            synchronized (lock) {
                latest.hasTarget = false;
                latest.distanceIn = Double.NaN;
                latest.groundDistanceIn = Double.NaN;         // <-- NEW
                latest.bearingXDeg = Double.NaN;
                latest.bearingYDeg = Double.NaN;
            }
        }

        if (SHOW_MASK) {
            Imgproc.cvtColor(morph, input, Imgproc.COLOR_GRAY2BGR);
        }
        return input;
    }

    public Result getLatestResult() {
        synchronized (lock) {
            Result r = new Result();
            r.hasTarget   = latest.hasTarget;
            r.cx          = latest.cx;  r.cy = latest.cy;
            r.radiusPx    = latest.radiusPx;
            r.diameterPx  = latest.diameterPx;
            r.distanceIn  = latest.distanceIn;
            r.groundDistanceIn = latest.groundDistanceIn; // <-- NEW
            r.bearingXDeg = latest.bearingXDeg;
            r.bearingYDeg = latest.bearingYDeg;
            return r;
        }
    }

    // ----- helpers -----
    private static int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }
    private static int makeOdd(int n) { return (n % 2 == 1) ? n : (n+1); }

    private static double focalLengthPx(double hfovDeg, int widthPx) {
        double hfov = Math.toRadians(hfovDeg);
        return (widthPx / 2.0) / Math.tan(hfov / 2.0);
    }
    private static double fovVFromH(double hfovDeg, int w, int h) {
        double hfov = Math.toRadians(hfovDeg);
        double ar = (double) h / (double) w;
        double vfov = 2.0 * Math.atan(Math.tan(hfov/2.0) * ar);
        return Math.toDegrees(vfov);
    }
}
