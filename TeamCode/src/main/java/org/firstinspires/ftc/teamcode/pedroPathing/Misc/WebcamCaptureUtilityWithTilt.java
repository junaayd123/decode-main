package org.firstinspires.ftc.teamcode.pedroPathing.Misc;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@TeleOp(name = "WEBCAM CAPTURE WITH TILT", group = "Utility")
public class WebcamCaptureUtilityWithTilt extends LinearOpMode
{
    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Servo constants
    final double SERVO_MIN_DEGREES = 0;
    final double SERVO_MAX_DEGREES = 180;
    final double DEGREES_PER_STEP = 1.0;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;

    @Override
    public void runOpMode()
    {
        VisionPortal portal;
        Servo camTilt = hardwareMap.get(Servo.class, "cam_tilt");

        // Start servo at 90 degrees (center)
        double currentDegrees = 90.0;
        camTilt.setPosition(degreesToServoPos(currentDegrees));

        if (USING_WEBCAM)
        {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .build();
        }
        else
        {
            portal = new VisionPortal.Builder()
                    .setCamera(INTERNAL_CAM_DIR)
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .build();
        }

        boolean lastRightBumper = false;
        boolean lastLeftBumper = false;

        waitForStart();

        while (!isStopRequested())
        {
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper  = gamepad1.left_bumper;

            if (rightBumper && !lastRightBumper) {
                currentDegrees = Math.min(currentDegrees + DEGREES_PER_STEP, SERVO_MAX_DEGREES);
                camTilt.setPosition(degreesToServoPos(currentDegrees));
            }
            if (leftBumper && !lastLeftBumper) {
                currentDegrees = Math.max(currentDegrees - DEGREES_PER_STEP, SERVO_MIN_DEGREES);
                camTilt.setPosition(degreesToServoPos(currentDegrees));
            }

            lastRightBumper = rightBumper;
            lastLeftBumper  = leftBumper;

            // --- Frame Capture ---
            boolean x = gamepad1.x;
            if (x && !lastX)
            {
                portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
                capReqTime = System.currentTimeMillis();
            }
            lastX = x;

            // --- Telemetry ---
            telemetry.addLine("######## Camera Capture Utility ########");
            telemetry.addLine(String.format(Locale.US, " > Resolution: %dx%d", RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
            telemetry.addLine(" > Press X (or Square) to capture a frame");
            telemetry.addLine(" > Right Trigger: tilt up  |  Left Trigger: tilt down");
            telemetry.addData(" > Camera Status", portal.getCameraState());
            telemetry.addData(" > Tilt (degrees)", String.format(Locale.US, "%.0f°", currentDegrees));
            telemetry.addData(" > Tilt (servo pos)", String.format(Locale.US, "%.4f", degreesToServoPos(currentDegrees)));

            if (capReqTime != 0)
                telemetry.addLine("\nCaptured Frame!");

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000)
                capReqTime = 0;

            telemetry.update();
        }
    }

    /**
     * Converts degrees (0–180) to a servo position (0.0–1.0).
     */
    private double degreesToServoPos(double degrees) {
        return degrees / 180.0;
    }
}