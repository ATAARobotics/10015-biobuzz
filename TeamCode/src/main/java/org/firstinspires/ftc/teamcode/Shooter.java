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
    public static int DATA_POINTS = 30;

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

    public static double RPM_DROP = 150.0;
    public static double FF = 0.00033; //November 16 2026
    public static double KS = 0.1;
    public static double P = 0.0014; //November 16 2026
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
    double averageRpm = 0.0;
    public static double SHOOT_RPM = 2400; // nov 16
    private boolean shooting = false;
    double operatorTweak = 0;
    int shotCounter = 0;
    boolean shotCounterReady = true;
    double rpmBeforeShot = 0.0;
    double lastShotTime = 0.0;
    public double error;
    double motorPower;
    double sum;
    boolean firstShot = true;
    HyperTelemetry telem;
    int loopCounter = 0;
    public static double RPM_TOLERANCE = 200;

    public Shooter(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
        rpmController = new PIDController(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(KS, FF);
        motor = new MotorEx(hardwareMap, "shooter"); // port 2 on expansion
        // motor2 = new MotorEx(hardwareMap, "encoder");
        recentVelocity = new LinkedList<Shooter.VelocityData>();

        timeSinceStart = new Timing.Stopwatch();
    }


    public void start() {
        timeSinceStart.start();
    }

    public void rpmUp() {
        operatorTweak += 50;
    }

    public void rpmDown() {
        operatorTweak -= 50;
    }

    public void toggleRpm() {
        if (shooting) {
            targetRpm = 0;
            shooting = false;
        } else {
            targetRpm = SHOOT_RPM + operatorTweak;
            shooting = true;
        }
    }
    public void setRpm(){
        targetRpm = SHOOT_RPM + operatorTweak;
        shooting = true;
    }
    public void stopRpm(){
        targetRpm =0;
        shooting = false;
    }

    public boolean isAtSpeed() {
        if (targetRpm <= 0 || recentVelocity.isEmpty()) {
            return false;
        }

        return Math.abs(targetRpm - averageRpm) <= RPM_TOLERANCE;
	/*	
        double measuredRpm = recentVelocity.getLast().velocity;
        return Math.abs(targetRpm - measuredRpm) <= RPM_TOLERANCE;
	*/
    }

    @Override
    public void periodic() {
        rpmController.setPID(P, I, D);
        rpmFeedforward = new SimpleMotorFeedforward(0.0, FF);

        double v = motor.getVelocity() / 28 * 60; //this gives us rpm
        double time = timeSinceStart.elapsedTime();
        recentVelocity.addLast(new VelocityData(time, v)); //recent rpm
        while (recentVelocity.size() > DATA_POINTS) {
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

	averageRpm = total / totalAmount;
	
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
        if (recentVelocity.size() >= 20) {
            double dif = recentVelocity.getLast().velocity - recentVelocity.get(recentVelocity.size() - 20).velocity; //recent - 5 ago
            if (dif < -RPM_DROP && recentVelocity.get(recentVelocity.size() - 20).time > lastShotTime && shotCounterReady) {
                rpmBeforeShot = recentVelocity.get(recentVelocity.size() - 20).velocity;
                shotCounter += 1; //adds a shot to the counter
                shotCounterReady = false; //makes it false because its no longer ready
                lastShotTime = recentVelocity.getLast().time;
            }
            if (!shotCounterReady && recentVelocity.getLast().velocity >= rpmBeforeShot - 100) { //rpmbeforeshot can be our target rpm that we want to get to, subtract by 100 for leeway
                //checks if our current rpm has made it back to rpm before we shot
                shotCounterReady = true; //since current rpm is greater than/equal to the rpm before we shot
            } else if (targetRpm - recentVelocity.getLast().velocity < 100 && firstShot) {
                shotCounterReady = true;
                firstShot = false;
            }
            rpmController.setSetPoint(targetRpm);

            motorPower = rpmController.calculate(v); //calculates error between setpoint/targetRpm and
            //current rpm, then finds the pid output

            if (targetRpm < 0) {
                targetRpm = 0;
            }
            // add in feed forward
            motorPower += rpmFeedforward.calculate(targetRpm);

            // Clamp power between 0.0 and 1.0
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));
            motor.set(motorPower);

            telem.log("shooter-motorPower", motorPower);
            telem.log("shooter-rpm-instant", v);
            telem.log("shooter-rpm-average", averageRpm);
            telem.log("shooter-rpm-target", targetRpm);
        }
    }
}
