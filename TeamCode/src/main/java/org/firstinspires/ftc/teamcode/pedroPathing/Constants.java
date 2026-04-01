package org.firstinspires.ftc.teamcode.pedroPathing;

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

public class Constants {
    public static double brakingStrength = 1.0;
    public static double brakingStart = 0.25;

    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(11.86) // kilograms!
        .forwardZeroPowerAcceleration(-31.7)
        .lateralZeroPowerAcceleration(-59.6)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.0))
        .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.06, 0.0))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0.0,0.0,0.0,0.0))
        .useSecondaryDrivePIDF(true)
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2,0,0.01,0))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.0,0.0,0.0,0.0))
        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0,0,0))
        ;

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("fr")
        .rightRearMotorName("br")
        .leftRearMotorName("bl")
        .leftFrontMotorName("fl")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(75.2)
        .yVelocity(62.5)
        ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .distanceUnit(DistanceUnit.INCH)
        .forwardPodY(5.693)
        .strafePodX(1.899)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        ;

    public static PathConstraints pathConstraints = new PathConstraints(
        0.99,  // tValue
        100,   // timeout
        brakingStrength, //1,     // braking strength
        brakingStart //1      // braking start
        );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}
