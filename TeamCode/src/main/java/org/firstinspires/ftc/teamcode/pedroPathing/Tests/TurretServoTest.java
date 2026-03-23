package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Servo Test", group = "Test")
public class TurretServoTest extends OpMode {

    private Servo turret;
    private Servo turret2;

    // How much to move the servo per loop tick when a button is held
    private static final double STEP = 0.01;

    private double turretPos  = 0.5;
    private double turret2Pos = 0.5;

    @Override
    public void init() {
        turret  = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        turret.setPosition(turretPos);
        turret2.setPosition(turret2Pos);

        telemetry.addLine("Turret Servo Test ready.");
        telemetry.addLine("D-Pad UP/DOWN  → turret");
        telemetry.addLine("D-Pad LEFT/RIGHT → turret2");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- turret: dpad up = increase, dpad down = decrease ---
        if (gamepad1.dpadUpWasPressed()) {
            turretPos += STEP;
        } else if (gamepad1.dpadDownWasPressed()) {
            turretPos -= STEP;
        }

        // --- turret2: dpad right = increase, dpad left = decrease ---
        if (gamepad1.dpadRightWasPressed()) {
            turret2Pos += STEP;
        } else if (gamepad1.dpadLeftWasPressed()) {
            turret2Pos -= STEP;
        }
        if(gamepad1.a){
            turretPos  = 0.08;
            turret2Pos = 0.08;
        }
        else if(gamepad1.b){
            turretPos  = 0.52;
            turret2Pos = 0.52;
        }
        else if(gamepad1.x){
            turretPos  = .95;
            turret2Pos = .95;
        }

        // Clamp both positions to [0.0, 1.0]
        turretPos  = Math.max(0.0, Math.min(1.0, turretPos));
        turret2Pos = Math.max(0.0, Math.min(1.0, turret2Pos));

        turret.setPosition(turretPos);
        turret2.setPosition(turret2Pos);

        // Telemetry
        telemetry.addLine("=== Turret Servo Test ===");
        telemetry.addData("turret  (UP/DOWN)    ", "%.3f", turretPos);
        telemetry.addData("turret2 (LEFT/RIGHT) ", "%.3f", turret2Pos);
        telemetry.addLine("");
        telemetry.addLine("Range: 0.0 – 1.0  |  Step: " + STEP + " per tick");
        telemetry.update();
    }
}