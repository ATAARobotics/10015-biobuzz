package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        //c.manualBrakeMode.set(true) add for teleop usage if you want to have manual brake mode enabled.
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.162211741049459);
        c.yPodOffset.set(2.1510477141132505);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.36213095182956506);
                Controller secondaryTranslationalForward = Controller.proportional(0.13379771038378013);
                Controller primaryTranslationalLateral = Controller.proportional(0.48367238758993353);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17870402325026);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015935710108248187));
                c.brake.set(Controller.proportionalFeedforward(0.013545353592010959));

                c.headingFeedback.set(Controller.proportional(6.751419995130347));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.052370394780099416, 0.007307163918109171));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09443158091786687, 0.0636100778339165));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.672292195322645E-4, 0.0015835156664673084));

                c.maxAchievableForwardVelocity.set(65.69256368160445);
                c.maxAchievableStrafeVelocity.set(51.84124669814729);
                c.naturalForwardDeceleration.set(34.18219937082535);
                c.naturalStrafeDeceleration.set(67.7287676126148);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}