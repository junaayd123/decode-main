/*package org.firstinspires.ftc.teamcode.pedroPathing;


import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ArtifactRampCV extends OpenCvPipeline {

    // ===== Camera assumptions =====
    public static int IMG_W = 640;
    public static int IMG_H = 480;
    public static double HFOV_DEG = 60.0;

    // pixels/inch: 640 / (2*tan(HFOV/2)*12)
    public static double PIXELS_PER_INCH =
            640.0 / (2.0 * Math.tan(Math.toRadians(HFOV_DEG / 2.0)) * 12.0);

    // ===== HSV bounds for PURPLE =====
    public static int PURPLE_H_LO = 120, PURPLE_S_LO = 100, PURPLE_V_LO = 80;
    public static int PURPLE_H_HI = 150, PURPLE_S_HI = 255, PURPLE_V_HI = 255;

    // ===== HSV bounds for GREEN =====
    public static int GREEN_H_LO = 40, GREEN_S_LO = 100, GREEN_V_LO = 80;
    public static int GREEN_H_HI = 80, GREEN_S_HI = 255, GREEN_V_HI = 255;

    // ===== Filters =====
    public static int BLUR_SIZE = 5;          // odd kernel (3/5/7)
    public static int MIN_AREA = 100;         // px^2
    public static double ASPECT_MAX = 2.5;    // keep contours where max(w/h, h/w) < ASPECT_MAX
    public static boolean SHOW_MASK = false;  // return mask for tuning instead of camera
    public static class Artifact {
        public int x, y, w, h;
        public int cx, cy;
        public double area;
        public double wIn, hIn;
    }

    // ===== Result snapshot =====
    public static class Result {
        public List<Artifact>
        public boolean hasTarget = false;
        public int x=0, y=0, w=0, h=0;   // boundingRect
        public int cx=0, cy=0;           // center
        public double area=0;            // contour area
        public double wIn=0, hIn=0;      // inches (from bbox via PIXELS_PER_INCH)
    }

    public static class DualResult {
        public Result purple = new Result();
        public Result green  = new Result();
    }

    private final Object lock = new Object();
    private final DualResult latest = new DualResult();

    // Reusable Mats
    private final Mat hsv = new Mat();
    private final Mat maskPurple = new Mat();
    private final Mat maskGreen = new Mat();
    private final Mat blurred = new Mat();
    private final Mat morph = new Mat();
    private final Mat hierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // BGR -> HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // Purple mask
        Core.inRange(hsv,
                new Scalar(clamp(PURPLE_H_LO,0,179), clamp(PURPLE_S_LO,0,255), clamp(PURPLE_V_LO,0,255)),
                new Scalar(clamp(PURPLE_H_HI,0,179), clamp(PURPLE_S_HI,0,255), clamp(PURPLE_V_HI,0,255)),
                maskPurple);

        // Green mask
        Core.inRange(hsv,
                new Scalar(clamp(GREEN_H_LO,0,179), clamp(GREEN_S_LO,0,255), clamp(GREEN_V_LO,0,255)),
                new Scalar(clamp(GREEN_H_HI,0,179), clamp(GREEN_S_HI,0,255), clamp(GREEN_V_HI,0,255)),
                maskGreen);

        // Process each color separately
        Result purpleResult = processMask(maskPurple, input, new Scalar(200,0,200)); // purple overlay
        Result greenResult  = processMask(maskGreen, input, new Scalar(0,200,0));    // green overlay

        synchronized (lock) {
            latest.purple = purpleResult;
            latest.green  = greenResult;
        }

        if (SHOW_MASK) {
            // Show combined mask instead of camera
            Mat combined = new Mat();
            Core.bitwise_or(maskPurple, maskGreen, combined);
            Imgproc.cvtColor(combined, input, Imgproc.COLOR_GRAY2BGR);
        }

        return input;
    }

    private Result processMask(Mat mask, Mat input, Scalar drawColor) {
        // Optional blur (Gaussian)
        int k = makeOdd(BLUR_SIZE);
        if (k >= 3) {
            Imgproc.GaussianBlur(mask, blurred, new Size(k,k), 0);
        } else {
            mask.copyTo(blurred);
        }

        // Morph open+close
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));
        Imgproc.morphologyEx(blurred, morph, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, kernel);

        // Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Result res = new Result();
        MatOfPoint best = null;
        double bestArea = 0;

        for (MatOfPoint c : contours) {
            double a = Imgproc.contourArea(c);
            if (a < MIN_AREA) continue;

            RotatedRect rr = Imgproc.minAreaRect(new MatOfPoint2f(c.toArray()));
            Size sz = rr.size;
            if (sz.width <= 0 || sz.height <= 0) continue;

            double aspect = Math.max(sz.width/sz.height, sz.height/sz.width);
            if (aspect < ASPECT_MAX && a > bestArea) {
                bestArea = a;
                best = c;
            }
        }

        if (best != null) {
            Rect br = Imgproc.boundingRect(best);
            int cx = br.x + br.width/2;
            int cy = br.y + br.height/2;
            double wIn = br.width  / PIXELS_PER_INCH;
            double hIn = br.height / PIXELS_PER_INCH;

            // Draw
            Imgproc.drawContours(input, List.of(best), -1, drawColor, 2);
            Imgproc.rectangle(input, br, drawColor, 2);
            Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0,0,255), -1);
            Imgproc.putText(input,
                    String.format("W: %.1fin H: %.1fin", wIn, hIn),
                    new Point(br.x, Math.max(15, br.y-8)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, drawColor, 1);

            // Save result
            res.hasTarget = true;
            res.x=br.x; res.y=br.y; res.w=br.width; res.h=br.height;
            res.cx=cx; res.cy=cy;
            res.area=bestArea;
            res.wIn=wIn; res.hIn=hIn;
        }

        return res;
    }

    public DualResult getLatestResult() {
        synchronized (lock) {
            DualResult r = new DualResult();
            r.purple = latest.purple;
            r.green  = latest.green;
            return r;
        }
    }

    private static int clamp(int v, int lo, int hi) { return Math.max(lo, Math.min(hi, v)); }
    private static int makeOdd(int n) { return (n % 2 == 1) ? n : (n+1); }
}

 */