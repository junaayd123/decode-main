package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;

import java.util.ArrayList;

@TeleOp(name = "Motor Power + Velocity Test", group = "Teleop Test")
public class MotorVelocityTest extends OpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private double coeff = 1;

    // single encoder port — replug wire between motors
    private DcMotorEx encoderMotor;

    private MultipleTelemetry telemetryA;
    private Timer runTimer;

    private boolean running = false;
    private boolean done    = false;
    private String activeMotorName = "";

    private ArrayList<Double> ampSamples = new ArrayList<>();
    private ArrayList<Double> velSamples = new ArrayList<>();
    private double avgAmp, avgVel;

    // averages per motor (stored after each run)
    private double avgAmpLF, avgVelLF;
    private double avgAmpLR, avgVelLR;
    private double avgAmpRF, avgVelRF;
    private double avgAmpRR, avgVelRR;
    private boolean hasLF, hasLR, hasRF, hasRR;

    // button edge detection
    private boolean prevA, prevB, prevX, prevY;

    @Override
    public void init() {
        leftFront    = hardwareMap.get(DcMotorEx.class, "lfmotor");
        leftRear     = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rightFront   = hardwareMap.get(DcMotorEx.class, "rfmotor");
        rightRear    = hardwareMap.get(DcMotorEx.class, "rbmotor");
        encoderMotor = hardwareMap.get(DcMotorEx.class, "depo1");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runTimer = new Timer();

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Ready. A=LF  B=RF  X=RR  Y=LR");
        telemetryA.update();
    }

    @Override
    public void loop() {
        boolean curA = gamepad1.a;
        boolean curB = gamepad1.b;
        boolean curX = gamepad1.x;
        boolean curY = gamepad1.y;

        boolean aJustPressed = curA && !prevA;
        boolean bJustPressed = curB && !prevB;
        boolean xJustPressed = curX && !prevX;
        boolean yJustPressed = curY && !prevY;

        prevA = curA; prevB = curB; prevX = curX; prevY = curY;

        // --- Start a run for the corresponding motor ---
        if (!running) {
            DcMotorEx motorToRun = null;
            String name = "";

            if (aJustPressed) { motorToRun = leftFront;  name = "leftFront";  }
            if (bJustPressed) { motorToRun = rightFront; name = "rightFront"; }
            if (xJustPressed) { motorToRun = rightRear;  name = "rightRear";  }
            if (yJustPressed) { motorToRun = leftRear;   name = "leftRear";   }

            if (motorToRun != null) {
                running = true;
                done    = false;
                activeMotorName = name;
                ampSamples.clear();
                velSamples.clear();
                motorToRun.setPower(coeff);
                runTimer.startTimer();
            }

            if (!running) {
                if (gamepad1.dpadDownWasPressed()) coeff -= 0.05;
                if (gamepad1.dpadUpWasPressed())   coeff += 0.05;
            }
        }

        // --- Run logic ---
        if (running) {
            double elapsed = runTimer.timer.seconds() - runTimer.curtime;

            if (elapsed >= 7.0) {
                // stop whichever motor was running
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                runTimer.stopTimer();
                computeAverages();
                saveAverages();
                running = false;
                done    = true;
            } else if (elapsed >= 1.0) {
                ampSamples.add(getActiveMotorAmps());
                velSamples.add(encoderMotor.getVelocity());
            }
        }

        // --- Telemetry ---
        if (running) {
            double elapsed = runTimer.timer.seconds() - runTimer.curtime;
            String phase = elapsed < 1.0 ? "Warming up..." : "Recording...";
            telemetryA.addData("Motor", activeMotorName);
            telemetryA.addData("Status", phase);
            telemetryA.addData("Elapsed (s)", String.format("%.2f", elapsed));
            telemetryA.addData("Samples", velSamples.size());
            telemetryA.addData("Live velocity (ticks/s)", encoderMotor.getVelocity());
            telemetryA.addData("Live amps", String.format("%.2f A", getActiveMotorAmps()));
        } else if (done) {
            telemetryA.addData("Finished motor", activeMotorName);
            telemetryA.addData("Samples recorded", velSamples.size());
            telemetryA.addData("Avg velocity (ticks/s)", String.format("%.2f", avgVel));
            telemetryA.addData("Avg amps", String.format("%.2f A", avgAmp));
            telemetryA.addLine("Press next motor button to continue.");
        } else {
            telemetryA.addLine("Press button to test a motor:");
            telemetryA.addLine("A=leftFront  B=rightFront  X=rightRear  Y=leftRear");
        }

        // always show accumulated results
        telemetryA.addLine("--- Results so far ---");
        if (hasLF) { telemetryA.addData("leftFront  vel / amps", String.format("%.1f ticks/s  /  %.2f A", avgVelLF, avgAmpLF)); }
        if (hasLR) { telemetryA.addData("leftRear   vel / amps", String.format("%.1f ticks/s  /  %.2f A", avgVelLR, avgAmpLR)); }
        if (hasRF) { telemetryA.addData("rightFront vel / amps", String.format("%.1f ticks/s  /  %.2f A", avgVelRF, avgAmpRF)); }
        if (hasRR) { telemetryA.addData("rightRear  vel / amps", String.format("%.1f ticks/s  /  %.2f A", avgVelRR, avgAmpRR)); }
        telemetryA.addData("power coeff (dpad up/down)", String.format("%.2f", coeff));
        telemetryA.update();
    }

    private double getActiveMotorAmps() {
        switch (activeMotorName) {
            case "leftFront":  return leftFront.getCurrent(CurrentUnit.AMPS);
            case "leftRear":   return leftRear.getCurrent(CurrentUnit.AMPS);
            case "rightFront": return rightFront.getCurrent(CurrentUnit.AMPS);
            case "rightRear":  return rightRear.getCurrent(CurrentUnit.AMPS);
            default:           return 0;
        }
    }

    private void computeAverages() {
        avgAmp = average(ampSamples);
        avgVel = average(velSamples);
    }

    private void saveAverages() {
        switch (activeMotorName) {
            case "leftFront":  avgAmpLF = avgAmp; avgVelLF = avgVel; hasLF = true; break;
            case "leftRear":   avgAmpLR = avgAmp; avgVelLR = avgVel; hasLR = true; break;
            case "rightFront": avgAmpRF = avgAmp; avgVelRF = avgVel; hasRF = true; break;
            case "rightRear":  avgAmpRR = avgAmp; avgVelRR = avgVel; hasRR = true; break;
        }
    }

    private double average(ArrayList<Double> list) {
        if (list.isEmpty()) return 0;
        double sum = 0;
        for (double v : list) sum += v;
        return sum / list.size();
    }
}