package org.firstinspires.ftc.teamcode.pedroPathing.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.C_Bot_Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.Deposition_C;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.IntakeManager;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.ShooterManager;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.TurretLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.lifters;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems_C_bot.regressions;

@Config
@TeleOp(name = "BLUE TELEOP", group = "A_TeleOp")
public class BotCTeleop_BlueNew extends OpMode {

    // --- HEADING LOCK PARAMETERS ---
    public static double HEADING_P               = -2.0;
    public static double HEADING_I               = -0.25;
    public static double HEADING_D               = -0.15;
    public static double TURN_DEADBAND           = 0.05;
    public static double TRANSLATION_DEADBAND    = 0.05;
    public static double INTEGRAL_MAX_ERR        = 0.05;
    public static double TURN_SETTLE_TIME        = 0.4;
    public static double TRANSLATION_SETTLE_TIME = 0.2;

    public static double Xp=0.3;
    public static double Xi=0.0;
    public static double Xd=0.001;
    public static double Yp=0.1;
    public static double Yi=0.0;
    public static double Yd=0.001;
    public static double Hp=2.0;
    public static double Hi=0.0;
    public static double Hd=0.07;

    // --- SUBSYSTEMS ---
    private Follower         follower;
    private TurretLimelight  turret;
    private lifters          lift;
    private Deposition_C     depo;
    private IntakeManager    intake;
    private VisionSubsystem  vision;
    private ShooterManager   shooter;
    private regressions      reg;

    private DcMotorEx            leftFront, leftRear, rightFront, rightRear;
    private GoBildaPinpointDriver pinpoint;

    private PIDController xPID = new PIDController(Xp, Xi, Xd);
    private PIDController yPID = new PIDController(Yp, Yi, Yd);
    private PIDController hPID = new PIDController(Hp, Hi, Hd);

    // --- DRIVE STATE ---
    private double  lockedHeading    = 0;
    private double  lastHeadingError = 0;
    private double  integralSum      = 0;
    private boolean wasTurning       = false;
    private boolean wasTranslating   = false;
    private boolean turningSettling  = false;
    private boolean transSettling    = false;
    private boolean firstLockLoop    = false; // suppresses derivative spike on lock entry
    private boolean wasShooting      = false;

    private ElapsedTime loopTimer        = new ElapsedTime();
    private ElapsedTime settleTimer      = new ElapsedTime();
    private ElapsedTime transSettleTimer = new ElapsedTime();
    private MultipleTelemetry telemetryA;

    // --- TELEOP STATE ---
    private enum Mode { nothing, findTag, faceGoal, faceRamp }
    private Mode mode               = Mode.nothing;
    private Mode modeBeforeRampScan = Mode.nothing;

    private double  distanceToGoal  = 0;
    private String  motif           = "gpp";
    private int     ballOnRamp;
    private int     greenInSlot;
    private boolean shootingTest    = false;
    private double  ourVelo         = 1300;
    private double  totalHedOffset  = 0;
    private double  speed           = 1.0;
    private boolean tagInitializing = false;

    private boolean frozen          = false;
    private Pose    holdPose        = null;

    private Pose    savedGatePose   = null;
    private boolean goingToGate     = false;
    private int rampFlag=0;

    private final Pose redGoal      = new Pose(-64, 134, 0);
    private final Pose redGoalFixed = new Pose(-72, 144, 0);
    private final Pose rampPose     = new Pose(-72, 83, 0);
    private final Pose startPose    = new Pose(53, 70, 0);

    private Timer  turretTimer        = new Timer();
    private Timer  rampScanDelayTimer = new Timer();
    private Servo  led, led2;

    // -----------------------------------------------------------------------
    // INIT
    // -----------------------------------------------------------------------
    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "lfmotor");
        leftRear   = hardwareMap.get(DcMotorEx.class, "lbmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rfmotor");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rbmotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        follower = C_Bot_Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        led  = hardwareMap.get(Servo.class, "led");
        led2 = hardwareMap.get(Servo.class, "led2");

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    // -----------------------------------------------------------------------
    // START
    // -----------------------------------------------------------------------
    @Override
    public void start() {
        turret  = new TurretLimelight(hardwareMap);
        lift    = new lifters(hardwareMap);
        depo    = new Deposition_C(hardwareMap);
        reg     = new regressions();
        intake  = new IntakeManager(hardwareMap, lift.sensors);
        vision  = new VisionSubsystem(hardwareMap);
        shooter = new ShooterManager(depo, lift, reg);
        pinpoint.update();
        lockedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        loopTimer.reset();
        VisionSubsystem.isRed =false;

        lift.allDown();
        lift.launchAngleServo.setPosition(0.04);
        lift.set_camera_tag_pos();
        motif = regressions.motif;
        shooter.isBlue = true;
    }

    // -----------------------------------------------------------------------
    // LOOP
    // -----------------------------------------------------------------------
    @Override
    public void loop() {
        // dt is measured at the very top before any subsystem work so it
        // reflects true loop period and doesn't corrupt the derivative term
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // --- SUBSYSTEM UPDATES ---
        vision.update();
        shooter.update();
        intake.update();
        turret.updateEncoderPos();
        follower.update();

        Pose cur = follower.getPose();
        distanceToGoal = cur.distanceFrom(redGoalFixed);

        // --- TELEOP LOGIC ---
        handleRampScan();
        handleGatePathing(cur);
        handleTurretMode(cur);
        handleTagLocalization();
        handleIntake();
        handleShooting();
        reg.RobY = cur.getY();

        // --- AUTO-UNFREEZE LOGIC ---
        boolean shootingNow = shooter.isShooting();
        if (wasShooting && !shootingNow && frozen) {
            frozen = false;
        }
        wasShooting = shootingNow;

        // --- HEADING LOCK DRIVE (dt passed in cleanly) ---
        if (!goingToGate) {
            handleHeadingLockDrive(dt);
        }

        updateLEDs();
        doTelemetry(cur);
    }

    // -----------------------------------------------------------------------
    // STOP
    // -----------------------------------------------------------------------
    @Override
    public void stop() {
        if (vision != null) vision.close();
    }

    // -----------------------------------------------------------------------
    // HEADING LOCK DRIVE
    // -----------------------------------------------------------------------
    private void handleHeadingLockDrive(double dt) {
        xPID.setPID(Xp, Xi, Xd);
        yPID.setPID(Yp, Yi, Yd);
        hPID.setPID(Hp, Hi, Hd);

        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        Pose curPose = follower.getPose();

        if (gamepad1.shareWasPressed()) {
            frozen = !frozen;
            if (frozen) {
                holdPose = new Pose(curPose.getX(), curPose.getY(), heading);
                xPID.reset();
                yPID.reset();
                hPID.reset();
            }
        }

        double forward, strafe, rotationOutput;

        if (frozen) {
            // Calculate PID outputs in field-relative terms
            // calculate(measured, target)
            double xPowerField = xPID.calculate(curPose.getX(), holdPose.getX());
            double yPowerField = yPID.calculate(curPose.getY(), holdPose.getY());

            // Calculate heading error and normalize it to [-pi, pi]
            double hError = holdPose.getHeading() - heading;
            while (hError > Math.PI) hError -= 2 * Math.PI;
            while (hError < -Math.PI) hError += 2 * Math.PI;

            // Calculate heading power (using target as current + error)
            double hPower = hPID.calculate(heading, heading + hError);

            // Rotate field-centric powers to robot-centric coordinates
            // Pedro Pathing: X+ Forward, Y+ Left, Heading CCW
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            forward = xPowerField * cos + yPowerField * sin;
            // Robot Y+ is Left, but motor mapping +strafe is Right.
            // So strafe_motor = -robot_y = x_field * sin - y_field * cos
            strafe  = xPowerField * sin - yPowerField * cos;
            // Motor mapping +turn is Clockwise. hPower is positive for CCW.
            rotationOutput = -hPower;

        } else {
            speed = gamepad1.cross ? 0.3 : 1.0;
            forward = -gamepad1.left_stick_y * speed;
            strafe  = (gamepad1.right_trigger - gamepad1.left_trigger) * speed;
            double turn = gamepad1.right_stick_x * speed;

            boolean driverTurning = Math.abs(gamepad1.right_stick_x) > TURN_DEADBAND;
            boolean driverTranslating = Math.abs(gamepad1.left_stick_y) > TRANSLATION_DEADBAND
                    || Math.abs(gamepad1.right_trigger) > TRANSLATION_DEADBAND
                    || Math.abs(gamepad1.left_trigger) > TRANSLATION_DEADBAND;

            if (transSettling && driverTranslating) transSettling = false;

            if (driverTurning) {
                rotationOutput = turn;
                wasTurning = true;
                turningSettling = false;
                transSettling = false;
                lockedHeading = heading;
                lastHeadingError = 0;
                integralSum = 0;
                firstLockLoop = false;

            } else if (wasTurning) {
                wasTurning = false;
                turningSettling = true;
                settleTimer.reset();
                rotationOutput = 0;

            } else if (turningSettling) {
                rotationOutput = 0;
                if (settleTimer.seconds() >= TURN_SETTLE_TIME) {
                    lockedHeading = heading;
                    integralSum = 0;
                    lastHeadingError = 0;
                    turningSettling = false;
                    firstLockLoop = true; // next LOCKED loop skips derivative
                }

            } else if (wasTranslating && !driverTranslating) {
                transSettling = true;
                transSettleTimer.reset();
                rotationOutput = 0;

            } else if (transSettling) {
                rotationOutput = 0;
                if (transSettleTimer.seconds() >= TRANSLATION_SETTLE_TIME) {
                    lockedHeading = heading;
                    integralSum = 0;
                    lastHeadingError = 0;
                    transSettling = false;
                    firstLockLoop = true; // next LOCKED loop skips derivative
                }

            } else {
                // LOCKED — run PID
                double headingError = lockedHeading - heading;
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                if (Math.abs(headingError) < INTEGRAL_MAX_ERR) {
                    integralSum += headingError * dt;
                } else {
                    integralSum = 0;
                }

                // Skip derivative on the very first loop after entering LOCKED
                // to prevent a spike from lastHeadingError being stale
                double derivative = 0;
                if (!firstLockLoop && dt > 0) {
                    derivative = (headingError - lastHeadingError) / dt;
                }
                firstLockLoop = false;
                lastHeadingError = headingError;

                rotationOutput = headingError * HEADING_P
                        + integralSum * HEADING_I
                        + derivative * HEADING_D;
            }

            wasTranslating = driverTranslating;
        }

        double lf = forward + strafe + rotationOutput;
        double lr = forward - strafe + rotationOutput;
        double rf = forward - strafe - rotationOutput;
        double rr = forward + strafe - rotationOutput;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lr)),
                Math.max(Math.abs(rf), Math.abs(rr))));
        leftFront.setPower(lf / max);
        leftRear.setPower(lr / max);
        rightFront.setPower(rf / max);
        rightRear.setPower(rr / max);
    }

    private void handleGatePathing(Pose cur) {
        if (gamepad1.circleWasPressed()) {
            savedGatePose = new Pose(cur.getX(), cur.getY(), cur.getHeading());
        }

        if (gamepad1.squareWasPressed() && savedGatePose != null) {
            PathChain gateChain = follower.pathBuilder()
                    .addPath(new BezierLine(cur, savedGatePose))
                    .setLinearHeadingInterpolation(cur.getHeading(), savedGatePose.getHeading())
                    .build();
            follower.followPath(gateChain);
            goingToGate = true;
        }

        if (goingToGate) {
            // Check for manual override
            double stickMag = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            boolean override = stickMag > 0.15 || Math.abs(gamepad1.right_stick_x) > 0.15 ||
                    gamepad1.right_trigger > 0.15 || gamepad1.left_trigger > 0.15;

            if (override || !follower.isBusy()) {
                follower.breakFollowing();
                // We do NOT call startTeleopDrive() here because we want the follower
                // to stay in IDLE mode and not fight our manual motor writes.
                goingToGate = false;

                pinpoint.update();
                lockedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
                firstLockLoop = true; // Prevents derivative spike on resumption
                integralSum = 0;
                lastHeadingError = 0;
            }
        }
    }

    // -----------------------------------------------------------------------
    // RAMP SCAN
    // -----------------------------------------------------------------------
    private void handleRampScan() {
        if (gamepad2.circleWasPressed() && rampFlag==0) {
            lift.set_camera_ramp_pos();
            lift.allDown();
//            modeBeforeRampScan = mode;
            mode = Mode.faceRamp;
            rampFlag = 1;
//            vision.startRampScan();
            rampScanDelayTimer.startTimer();
            shooter.setMotifShot(true);
        }


//        if (vision.isRampScanning()) {
//            if (rampScanDelayTimer.checkAtSecondsBigWindow(1.0)) {
//                lift.allDown();
//            }
//        }
        if(rampScanDelayTimer.checkAtSecondsBigWindow(0.2) && rampFlag==1){
            vision.startRampScan();
            rampFlag =2;
        }
        else if (rampScanDelayTimer.checkAtSecondsBigWindow(2.0) && rampFlag==2) {
            rampFlag = 3;
            mode        =  Mode.faceGoal;
            vision.finishRampScan();
            ballOnRamp  = vision.getRampBallVerdict();
            greenInSlot = getGreenPos();
        }
        else if(rampScanDelayTimer.checkAtSecondsOpenEnd(2.2) && rampFlag==3){
            rampScanDelayTimer.stopTimer();
            shooter.startShooting(motif, (ballOnRamp % 3), greenInSlot);
            rampFlag =0;
        }
//        else if (rampScanDelayTimer.timerIsOn()) {
//            if (rampScanDelayTimer.checkAtSecondsBigWindow(1.0)) {
//                mode        =  Mode.faceGoal;
//                ballOnRamp  = vision.getRampBallVerdict() % 3;
//                greenInSlot = getGreenPos();
//            }
//            if (rampScanDelayTimer.checkAtSecondsBigWindow(1.3)) {
////                rampFlag=0;
//                rampScanDelayTimer.stopTimer();
////                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
//                shooter.startShooting(motif, ballOnRamp, greenInSlot);
//            }
//        }
    }

    // -----------------------------------------------------------------------
    // TURRET MODE
    // -----------------------------------------------------------------------
    private void handleTurretMode(Pose cur) {
        Pose target = redGoal;
        double headingToTarget = calculateHeadingTo(cur, target);
        double robHeading = normalizeAngle(follower.getTotalHeading() - totalHedOffset);

        switch (mode) {
            case faceGoal:
                if (distanceToGoal > 125) {
                    turret.toTargetInDegrees2(Math.toDegrees(robHeading) + reg.getBlueTurretFar(cur.getX(),cur.getY()));
                } else {
                    turret.toTargetInDegrees2(Math.toDegrees(robHeading - headingToTarget));
                }
                break;
            case findTag:
                turret.toTargetInDegrees();
                break;
            case faceRamp:
                double headingToRamp = calculateHeadingTo(cur, rampPose);
                double rampDeg = Math.toDegrees(robHeading - headingToRamp);
                turret.toTargetInDegrees2(rampDeg);
                break;
            case nothing:
                if (gamepad1.dpad_left) {
                    turret.toTargetInDegrees2((turret.currentPos / (670.0 / 180.0)) - 1.5);
                } else if (gamepad1.dpad_right) {
                    turret.toTargetInDegrees2((turret.currentPos / (670.0 / 180.0)) + 1.5);
                }
                break;
        }
    }

    // -----------------------------------------------------------------------
    // TAG LOCALIZATION
    // -----------------------------------------------------------------------
    private void handleTagLocalization() {
        if (gamepad1.triangleWasPressed()) {
            lift.set_camera_tag_pos();
            if (tagInitializing || mode == Mode.findTag) {
                tagInitializing = false;
                mode = Mode.nothing;
                vision.setAprilTagEnabled(false);
            } else {
                turretTimer.startTimer();
                mode = Mode.findTag;
                turret.setDegreesTarget(0);
                vision.setAprilTagEnabled(true);
            }
        }

        if (turretTimer.checkAtSecondsOpenEnd(0.5)) {
            tagInitializing = true;
            turretTimer.stopTimer();
        }

        if (tagInitializing) {
            Pose localized = vision.getLocalizedPose(turret.currentPos);
            if (localized != null) {
                mode           = Mode.faceGoal;
                follower.setPose(localized);
                totalHedOffset = follower.getTotalHeading() - localized.getHeading();
                tagInitializing = false;
                vision.setAprilTagEnabled(false);
            }
        }
    }

    // -----------------------------------------------------------------------
    // INTAKE
    // -----------------------------------------------------------------------
    private void handleIntake() {
        if (gamepad2.rightBumperWasPressed()) {
            lift.allDown();
            if (intake.isCollecting()) intake.stop();
            else intake.startCollecting();
        }

        if (shooter.isShooting()) intake.stop();

        if (gamepad2.left_bumper) {
            intake.manualReverse();
        } else if (gamepad2.leftBumperWasReleased()) {
            intake.manualStop();
        }
    }

    // -----------------------------------------------------------------------
    // SHOOTING
    // -----------------------------------------------------------------------
    private void handleShooting() {
        shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
        if (gamepad2.psWasPressed()) {
            if      (motif.equals("gpp")) motif = "pgp";
            else if (motif.equals("pgp")) motif = "ppg";
            else                          motif = "gpp";
        }

        if (gamepad2.dpadDownWasPressed()) {
            shooter.stop();
            intake.stop();
            rampFlag = 0;
            rampScanDelayTimer.stopTimer();
            mode = Mode.faceGoal;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            reg.turretOffset-=0.5;
        }
        else if (gamepad2.dpadRightWasPressed()) {
            reg.turretOffset+=0.5;
        }
        else if (gamepad2.dpadUpWasPressed()) {
            reg.turretOffset=0;
        }


        if (gamepad2.crossWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
//                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp  = 0;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
        if (gamepad2.squareWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
//                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp  = 0;
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);

                // Freeze position
                frozen = true;
                holdPose = follower.getPose();
                xPID.reset();
                yPID.reset();
                hPID.reset();
            }
        }
        if (gamepad2.triangleWasPressed()) {
            lift.allDown();
            if (!lift.checkNoBalls()) {
                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
                ballOnRamp  = 0;
                shooter.setMotifShot(true);
                greenInSlot = getGreenPos();
                shooter.startShooting(motif, ballOnRamp, greenInSlot);
            }
        }
        if (gamepad2.circleWasPressed()) {
//            lift.allDown();
//            if (!lift.checkNoBalls()) {
//                shooter.updateTarget(distanceToGoal, shootingTest, ourVelo);
//                ballOnRamp  = 2;
//                greenInSlot = getGreenPos();
//                shooter.startShooting(motif, ballOnRamp, greenInSlot);
//            }
        }

        if (!shootingTest && !shooter.isShooting()) {
            lift.set_angle_custom(reg.distanceToAngle(distanceToGoal));
        }

        if (gamepad1.dpadUpWasPressed())   ourVelo += 20;
        if (gamepad1.dpadDownWasPressed())  ourVelo -= 20;
    }

    // -----------------------------------------------------------------------
    // LEDs
    // -----------------------------------------------------------------------
    private void updateLEDs() {
        if (intake.isCollecting()) {
            led.setPosition(0.28); // RED
            led2.setPosition(0.28);
        } else if (intake.getBallCount() >= 3) {
            led.setPosition(0.5);  // GREEN
            led2.setPosition(0.5);
        } else if (mode == Mode.findTag) {
            led.setPosition(0.388); // YELLOW
            led2.setPosition(0.388);
        } else if (mode == Mode.faceGoal) {
            led.setPosition(0.611); // BLUE
            led2.setPosition(0.611);
        } else {
            led.setPosition(0);     // BLACK/OFF
            led2.setPosition(0);
        }
    }

    // -----------------------------------------------------------------------
    // HELPERS
    // -----------------------------------------------------------------------
    private double calculateHeadingTo(Pose cur, Pose target) {
        double raw     = Math.atan2(target.getY() - cur.getY(), target.getX() - cur.getX());
        double flipped = raw + Math.PI;
        flipped = ((flipped + Math.PI) % (2 * Math.PI)) - Math.PI;
        return flipped + Math.PI;
    }

    private double normalizeAngle(double angle) {
        while (angle >= Math.toRadians(210)) angle -= 2 * Math.PI;
        while (angle < -Math.PI)             angle += 2 * Math.PI;
        return angle;
    }

    private int getGreenPos() {
        if (lift.sensors.getLeft() == 1) return 0;
        return (lift.sensors.getRight() == 1) ? 2 : 1;
    }

    // -----------------------------------------------------------------------
    // TELEMETRY
    // -----------------------------------------------------------------------
    private void doTelemetry(Pose cur) {
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double headingErr = lockedHeading - heading;
        while (headingErr >  Math.PI) headingErr -= 2 * Math.PI;
        while (headingErr < -Math.PI) headingErr += 2 * Math.PI;

        telemetryA.addData("motif",                motif);
        telemetryA.addData("balls on ramp",                ballOnRamp);
        telemetryA.addLine(savedGatePose != null ?
                "SAVED POSE: (" + String.format("%.1f", savedGatePose.getX()) + ", " + String.format("%.1f", savedGatePose.getY()) + ")" : "NO SAVED POSE");
        telemetryA.addData("turret mode",          mode);
        telemetryA.addData("distance to goal",     distanceToGoal);
        telemetryA.addData("target velocity",      shootingTest ? ourVelo : depo.targetVelocity);
        telemetryA.addData("X",                    cur.getX());
        telemetryA.addData("y",                    cur.getY());
        telemetryA.addData("heading (deg)",        String.format("%.2f", Math.toDegrees(heading)));
        telemetryA.addData("locked heading (deg)", String.format("%.2f", Math.toDegrees(lockedHeading)));
        telemetryA.addData("heading error (deg)",  String.format("%.2f", Math.toDegrees(headingErr)));
        telemetryA.addData("integral sum",         String.format("%.4f", integralSum));
        telemetryA.addData("trans settle (s)",     String.format("%.2f", transSettleTimer.seconds()));
        telemetryA.addData("turn settle (s)",      String.format("%.2f", settleTimer.seconds()));
        telemetryA.addData("HEADING_P",            HEADING_P);
        telemetryA.addData("HEADING_I",            HEADING_I);
        telemetryA.addData("HEADING_D",            HEADING_D);
        telemetryA.addData("TURN_SETTLE_TIME",     TURN_SETTLE_TIME);
        telemetryA.addData("TRANS_SETTLE_TIME",    TRANSLATION_SETTLE_TIME);
        telemetryA.update();
    }
}