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
        .mass(12.474) // kilograms!
        .forwardZeroPowerAcceleration(-30.0)
        .lateralZeroPowerAcceleration(-68.5)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.025))
        .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0, 0.04, 0.02))
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
        .xVelocity(76.7)
        .yVelocity(58.4)
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
        0.99,  // tValue
        100,   // timeout
        0.60, //1,     // braking strength
        2 //1      // braking start
        );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}
