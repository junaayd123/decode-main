package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Depo Motor Direction Test", group = "tuning")
public class depo_motor_directions extends OpMode {

    private DcMotorEx depo;
    private DcMotorEx depo1;
    private DcMotorEx depo2;
    private ColorSensors_New colSensors;

    // Power to apply when a button is held
    private static final double TEST_POWER = 0.3;

    @Override
    public void init() {
        depo  = hardwareMap.get(DcMotorEx.class, "depo");
        depo1 = hardwareMap.get(DcMotorEx.class, "depo1");
        depo2 = hardwareMap.get(DcMotorEx.class, "depo2");
        colSensors= new ColorSensors_New(hardwareMap);

        depo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        depo2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        depo.setDirection(DcMotorSimple.Direction.REVERSE);
        depo1.setDirection(DcMotorSimple.Direction.FORWARD);
        depo2.setDirection(DcMotorSimple.Direction.REVERSE);

        depo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        depo2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Depo Motor Direction Test ready.");
        telemetry.addLine("X → depo  |  Y → depo1  |  B → depo2");
        telemetry.addLine("Hold button to spin. Release to stop.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Each button independently drives one motor at TEST_POWER
        // No buttons held = all motors stop
        depo.setPower (gamepad1.x ? TEST_POWER : 0);
        depo1.setPower(gamepad1.y ? TEST_POWER : 0);
        depo2.setPower(gamepad1.b ? TEST_POWER : 0);
        telemetry.addData("intensity left", colSensors.getIntLeft());
        telemetry.addData("intensity right", colSensors.getIntRight());
        telemetry.addData("intensity back", colSensors.getIntBack());
        telemetry.addLine("=== Depo Motor Direction Test ===");
        telemetry.addData("X  →  depo  ", gamepad1.x  ? "RUNNING" : "stopped");
        telemetry.addData("Y  →  depo1 ", gamepad1.y  ? "RUNNING" : "stopped");
        telemetry.addData("B  →  depo2 ", gamepad1.b  ? "RUNNING" : "stopped");
        telemetry.addLine("");
        telemetry.addData("Test power", TEST_POWER);
        telemetry.addData("velo 0", depo.getVelocity());
        telemetry.addData("velo 1", depo1.getVelocity());
        telemetry.addData("velo 2", depo2.getVelocity());
        telemetry.addLine("If a motor spins the wrong way,");
        telemetry.addLine("flip its Direction to REVERSE in shooterPID_test.");
        telemetry.update();
    }
}