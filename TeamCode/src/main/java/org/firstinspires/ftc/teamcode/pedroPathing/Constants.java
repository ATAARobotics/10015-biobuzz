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
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(12.474) // kg
        .forwardZeroPowerAcceleration(-26.246)
        .lateralZeroPowerAcceleration(-64.512)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.03))
        .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.07, 0.03))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.00001, 0.6, 0.02))
        .centripetalScaling(0.005)
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
        .xVelocity(80.988)
        .yVelocity(67.522)
        ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
        .distanceUnit(DistanceUnit.INCH)
        .forwardPodY(6.65)
        .strafePodX(0.886)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        ;

    public static PathConstraints pathConstraints = new PathConstraints(
        0.995,  // tValue (bezier t-value greater than this to be at end?)
        0.1,    // velocity (inches / second) default is 0.1
        0.5,    // translational (in inches) default is 0.1
        Math.toRadians(1),  // heading (in radians, default 0.007 is 0.4010705 degress)
        100,    // timeout
        1.0,    // braking strength
        BEZIER_CURVE_SEARCH_LIMIT,
        1.0     // braking start, "percent of predicted stopping distance"
        );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();
    }
}
