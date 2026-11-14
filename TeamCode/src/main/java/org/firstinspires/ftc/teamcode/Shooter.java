package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;

import java.util.LinkedList;

@Configurable
public class Shooter extends SubsystemBase {
    public static int dataPoints = 30;

    class VelocityData {
        public double time;
        public double velocity;

        public VelocityData(double t, double v) {
            time = t;
            velocity = v;
        }
    }

    PIDController rpmController;
    SimpleMotorFeedforward rpmFeedforward;

    public static double increaseRate = 50.0;
    public static double RPM_DROP = 150.0;
    public static double FF = 0.000171; //November 8 2026
    public static double P = 0.0011; //November 8 2026
    public static double I = 0.00012; //November 8 2026
    public static double D = 0.0;
    public static double AVERAGE_EXPONENT = 1.2;
    LinkedList<Shooter.VelocityData> recentVelocity = null;
    LinkedList<Double> rpmHistory = new LinkedList<>();
    public static int MAX_SAMPLES = 5;
    Timing.Stopwatch timeSinceStart;
    MotorEx motor;
    MotorEx motor2;
    double targetRpm = 0.0;
    public static double SHOOT_RPM = 2225;
    private boolean shooting = false;
    double operatorTweak = 50;
    int shotCounter = 0;
    double shootRpmTolerance = 0.0; //Todo: find a range that the shooter is good at shooting
    boolean shotCounterReady = true;
    double rpmBeforeShot = 0.0;
    double lastShotTime = 0.0;
    double rps;
    int ticks;
    int lastTicks;
    double lastTime;
    public double error;
    double motorPower;
    double sum;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    HyperTelemetry telem;
    int loopCounter = 0;
    Servo turret;

    public Shooter(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
        rpmController = new PIDController(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(0.0, FF);
        motor = new MotorEx(hardwareMap, "shooterL"); // port 2 on expansion
        motor2 = new MotorEx(hardwareMap, "encoder");
        turret = hardwareMap.get(Servo.class, "turret");//port 0 on expansion
        recentVelocity = new LinkedList<Shooter.VelocityData>();

        timeSinceStart = new Timing.Stopwatch();
    }


    public void start() {
        timeSinceStart.start();
    }

    public void rpmUp(){
        operatorTweak += 50;
    }
    public void rpmDown(){
        operatorTweak -= 50;
    }
    public void toggleRpm(){
        if (shooting){
            targetRpm = 0;
            shooting = false;
        } else{
            targetRpm = SHOOT_RPM + operatorTweak;
            shooting = true;
        }
    }


    @Override
    public void periodic() {
        //Deleted shot counter stuff because it doesn't work and is messy, it still can be found in shootertestbiobuzz
        rpmController.setPID(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(0.0, FF);
        //telem.logBoth("feedforward", FF);

        double v = motor.getVelocity() / 28 * 60; //this gives us rpm

        motorPower = rpmController.calculate(v); //calculates error between setpoint/targetRpm and
        //current rpm, then finds the pid output

        if (targetRpm < 0){targetRpm = 0;}
        // add in feed forward
        motorPower += rpmFeedforward.calculate(targetRpm);

        // Clamp power between 0.0 and 1.0
        motorPower = Math.max(0.0, Math.min(1.0, motorPower));
        motor.set(motorPower);

        telem.log("shooter-motorPower", motorPower);
        telem.log("RPM", v);
        telem.logBoth("targetRpm", targetRpm);
    }
}