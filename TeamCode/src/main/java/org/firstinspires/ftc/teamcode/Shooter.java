package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public static double RPM_DROP = 200.0;
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
    double sum;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    HyperTelemetry telem;
    int loopCounter = 0;

    public Shooter(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
        rpmController = new PIDController(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(0.0, FF);
        motor = new MotorEx(hardwareMap, "shooterL");
        motor2 = new MotorEx(hardwareMap, "encoder");
        recentVelocity = new LinkedList<Shooter.VelocityData>();

        timeSinceStart = new Timing.Stopwatch();
    }


    public void start() {
        timeSinceStart.start();
    }

    @Override
    public void periodic() {
        rpmController.setPID(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(0.0, FF);
        telem.logBoth("feedforward", FF);

        double v = motor.getVelocity() / 28 * 60; //this gives us rpm
        recentVelocity.addLast(new Shooter.VelocityData(timeSinceStart.elapsedTime(), v)); //recent rpm
        while (recentVelocity.size() > dataPoints) {
            recentVelocity.removeFirst();
        }
        double total = 0.0;
        double amount = 1;
        double totalAmount = 0.0;
        for (Shooter.VelocityData data : recentVelocity) {
            totalAmount = totalAmount + amount;
            total = total + (data.velocity * amount); // adding recentvelocity data
            amount *= AVERAGE_EXPONENT; //1.0 is equal to normal average, more than that
            //creates an exponential graph
        }
        double averageVelocity = total / totalAmount; //finding average - total value of data/number of data points
        telem.logBoth("Average-RPM", averageVelocity);

        rpmHistory.add(recentVelocity.getLast().velocity);
        while (rpmHistory.size() > MAX_SAMPLES) {
            rpmHistory.removeFirst();
        }
        sum = 0.0;
        for (double i : rpmHistory) {
            sum += Math.abs(i - targetRpm);
        }
        error = sum / rpmHistory.size(); //average error


        //Shot Counter
        if (recentVelocity.size() >= 5) {
            double dif = recentVelocity.getLast().velocity - recentVelocity.get(recentVelocity.size() - 5).velocity; //recent - 5 ago
            if (dif < -RPM_DROP && recentVelocity.get(recentVelocity.size() - 5).time > lastShotTime && shotCounterReady) {
                rpmBeforeShot = recentVelocity.get(recentVelocity.size() - 5).velocity;
                shotCounter += 1; //adds a shot to the counter
                shotCounterReady = false; //makes it false because its no longer ready
                lastShotTime = recentVelocity.getLast().time;
            }
            if (!shotCounterReady && recentVelocity.getLast().velocity >= rpmBeforeShot - 100) { //rpmbeforeshot can be our target rpm that we want to get to, subtract by 100 for leeway
                //checks if our current rpm has made it back to rpm before we shot
                shotCounterReady = true; //since current rpm is greater than/equal to the rpm before we shot
            }
        }
        telem.log("RPM", v);
        telem.logBoth("targetRpm", targetRpm);
        telem.logBoth("shotCounter", shotCounter);
    }
    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        GamepadEx operator;

        public HumanInputs(GamepadEx operator, GamepadEx driver) {
            this.operator = operator;
            this.driver = driver;
            addRequirements(Shooter.this);
        }

        @Override
        public void execute() {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                targetRpm += increaseRate;
            }
            // Decrease power by 0.1 on each new D-pad down press
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                targetRpm -= increaseRate;
            }

            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                targetRpm += 500;
            }

            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                targetRpm -= 500;
            }
        }
    }
}