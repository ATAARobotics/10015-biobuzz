package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Configurable
@TeleOp(name = "KalmanFilter")
public class KalmanFilter extends ShooterTestBiobuzz {
    MotorEx motor;
    double x = 0; // your initial state
    double Q = 0.1; // your model covariance
    double R = 0.4; // your sensor covariance
    double p = 1; // your initial covariance guess
    double K = 1;

    double x_previous = x;
    double p_previous = p;
    double lastLoopTime;
    public static double increaseRate;

    double u = 0;
    double z = 0;
    public void init() {

    }

    public void loop() {

        u = (targetRpm - x_previous)/increaseRate;
        x = x_previous + u;

        p = p_previous + Q;

        K = p/(p + R);

        z = motor.getVelocity()/28*60;

        x = x + K * (z - x);

        p = (1 - K) * p;

        x_previous = x;
        p_previous = p;
        lastLoopTime = time;

    }
}
