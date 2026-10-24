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
                Controller primaryTranslationalForward = Controller.proportional(0.30574640599131575);
                Controller secondaryTranslationalForward = Controller.proportional(0.11296512731935968);
                Controller primaryTranslationalLateral = Controller.proportional(0.5008336245039062);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1850446417332772);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01577738034395283));
                c.brake.set(Controller.proportionalFeedforward(0.013410773292359904));

                c.headingFeedback.set(Controller.proportional(5.727709390565341/2));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.053712816419025755, 0.0060265235344253675));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0801693877236864, 0.07372056657910407));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014248657119717719, 0.0012953592673826742));

                c.maxAchievableForwardVelocity.set(61.50756019242531);
                c.maxAchievableStrafeVelocity.set(49.99310127687052);
                c.naturalForwardDeceleration.set(30.229390802371324);
                c.naturalStrafeDeceleration.set(65.7377274910355);
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