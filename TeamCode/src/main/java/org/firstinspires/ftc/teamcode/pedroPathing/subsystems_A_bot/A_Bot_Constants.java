package org.firstinspires.ftc.teamcode.pedroPathing.subsystems_A_bot;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.247)
            .forwardZeroPowerAcceleration(-57.91943364261733)
            .lateralZeroPowerAcceleration(-72.72021922763408)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.1, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.0075, 0, 0.000005, 0.085, 0)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(4, 0, 0.1, 0))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.0075, 0, 0.000005, 0.085, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("flmotor")
            .leftRearMotorName("blmotor")
            .rightFrontMotorName("frmotor")
            .rightRearMotorName("brmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.5767)
            .yVelocity(55.4236);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.9367)
            .strafePodX(5.52)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            0.75,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}*/

public class A_Bot_Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-36.70498698)
            .lateralZeroPowerAcceleration(-68.37003861162188)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0003)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.000001, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.07, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.0075, 0, 0.000005, 0.085, 0)
            )
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.11, 0, 0, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lfmotor")
            .leftRearMotorName("lbmotor")
            .rightFrontMotorName("rfmotor")
            .rightRearMotorName("rbmotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.896702237104)
            .yVelocity(60.22467346211731);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.2)
            .strafePodX(-3.2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            //.customEncoderResolution(13.26291192)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            0.625,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
