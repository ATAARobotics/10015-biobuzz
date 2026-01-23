package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.LinkedList;

@Configurable
public class Shooter extends SubsystemBase {
    MotorEx motor0;
    MotorEx motor1;
    MotorGroup shooterMotor;
    Servo hood;
    double ticksPerSecond;
    double power;
    double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)

    // shot-counter
    int shotsFired = 0;

    class RpmData {
        public double time;
        public double rpm;
        public RpmData(double t, double r) {
            time = t;
            rpm = r;
        }
    }
    LinkedList<RpmData> recentRpms;
    public static double RPM_WINDOW_LENGTH = 0.150;  // seconds
    // the algorithm we use is:
    // - a bucket of recent samples, at most 200ms in length
    // - if the oldest rpm minus the newest rpm shows a >400 rpm drop, that's a shot
    // looking at telemetry data, we determined that this only happens
    // during shots (normal spin-down is slower so the 200ms window can't see it)

    //Tuned on November 27:
    public static double FAR_RPM = 4950;
    public static double NEAR_RPM = 4000;
    public static double HOOD_MAX = 0.75;
    public static double HOOD_MIN = 0.35;
    public static double MANUAL_RPM = 0;
    static double RPM_VS_DIST_SLOPE = 18.941;
    static double RPM_VS_DIST_INTERCEPT = 2327.9;
    static double HOOD_COEF = 0.0732;
    static double HOOD_EXP = 0.4768;

    public static int RPM_DROP_FOR_SHOT = 200;  // how many RPMs must drop for "a shot" to be counted

    double BAND = 10;
    double BANG_POWER = 1.0;
    public static double RPM_TOLERANCE = 200;  // jan22 changed from 250
    boolean powerOn = false;
    private static final double TICKS_PER_REV = 28.0;  // fixme: get from motor
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double targetRpm;
    double targetHood = HOOD_MIN;
    double currentRpm;
    double voltage; // current battery voltage
    public boolean autoRpm = false;
    public double aprilDistance;
    //public double geometricDistance;

    public static double kv = 0.002213; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 0.129514; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    public Shooter(HardwareMap hardwareMap) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor0.setInverted(true);

        motor1 = new MotorEx(hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(false);

        shooterMotor = new MotorGroup(motor0, motor1);
        targetRpm = 0;
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?

        hood = hardwareMap.get(Servo.class, "hood");
        recentRpms = new LinkedList<RpmData>();
    }

    public void reset() {
        stop();
    }

    public void stop() {
        targetRpm = 0;
        motor0.set(0);
        motor1.set(0);
    }
    public void read_sensors(double time) {
        // get any inputs from our encoders or other sensors
        ticksPerSecond = motor1.getVelocity();
        currentRpm = (ticksPerSecond * 60) / TICKS_PER_REV;
        voltage = battery.getVoltage();

        // recent RPM data for shot-counter.
        recentRpms.addLast(new RpmData(time, currentRpm));
        // ensure we only have 200ms or less worth of data
        while (time - recentRpms.getFirst().time > RPM_WINDOW_LENGTH) {
            recentRpms.removeFirst();
        }
    }


    // leave this here for easy copy-pasting when creating a new command
    public class CommandTemplate extends CommandBase {
        //public void initialize() {}
        //public void execute() {}
        //public boolean isFinished() { return false; }
        //public void end(boolean interrupted){}
    }

    public boolean readyToShoot() {
        return (targetRpm > 0 && Math.abs(currentRpm - targetRpm) < RPM_TOLERANCE);
    }

    public int getCurrentShots() {
        return shotsFired;
    }

    public void autoShootRpm() {
        autoRpm = true;
    }

    public void manualShootRpm() {
        autoRpm = false;
        targetRpm = 0;
    }

    @Override
    public void periodic() {
        // auto-computed RPM, optional
        if (autoRpm /*&& aprilDistance > 0.5*/) {
            targetRpm = RPM_VS_DIST_SLOPE * aprilDistance + RPM_VS_DIST_INTERCEPT;
            targetHood = HOOD_COEF *Math.pow(aprilDistance, HOOD_EXP);
        }
        if (MANUAL_RPM > 1.0 && targetRpm > 0.0) {
            targetRpm = MANUAL_RPM;
        }

        appliedVoltage = (kv * targetRpm) + ks;
        power = appliedVoltage / voltage;

        // hack: trying to make this actually be "bang-F" instead of "just F"
        power = power * 0.9;

        //power += velocity.calculate(currentRpm); //Change power to += when Feed Forward is used
        if (currentRpm < (targetRpm - BAND) && !powerOn) {
            powerOn = true;
        }
        else if (currentRpm > (targetRpm + BAND) && powerOn) {
            powerOn = false;
        }
        if (powerOn) power = BANG_POWER;
        if (targetRpm == 0) power = 0;
        if (power < 0) power = 0;

        if (targetHood < HOOD_MIN){
            targetHood = HOOD_MIN;
        }
        if (targetHood > HOOD_MAX){
            targetHood = HOOD_MAX;
        }
        hood.setPosition(targetHood);

        shooterMotor.set(power);

        // count shots
        if (recentRpms.size() > 2) {
            double minRpm = 10000.0; // we can't spin this fast
            double maxRpm = 0.0;
            for (RpmData r : recentRpms) {
                if (r.rpm < minRpm) { minRpm = r.rpm; }
                if (r.rpm > maxRpm) { maxRpm = r.rpm; }
            }
            //double rpmDrop = maxRpm - minRpm;// recentRpms.getFirst().rpm - recentRpms.getLast().rpm;
            double rpmDrop = recentRpms.getFirst().rpm - recentRpms.getLast().rpm;
            if (rpmDrop > RPM_DROP_FOR_SHOT) {
                shotsFired += 1;
                recentRpms.clear();
            }
        }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.logBoth("motor0", power); //what you see on the screen
        telem.logBoth("targetRpm", targetRpm);
        telem.logBoth("Current RPM", currentRpm);
        telem.logBoth("Applied Voltage", appliedVoltage);
        telem.logBoth("Shots Fired" , shotsFired);
       // pack.put("ticksPerSecond", ticksPerSecond);
        telem.log("shooter-rpm-target", targetRpm);
        telem.log("shooter-rpm-current", currentRpm);
        telem.log("shooter-power", power);
        telem.log("shooter-hood-angle", targetHood);
        telem.log("shooter-auto-rpm", autoRpm);
        telem.log("shooter-distance", aprilDistance);
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
            // decide what to do based on sensors and human inputs from controller

            // FIXME: need operator controls ... and far-shot target?
            // and "hood" controls?

           /* if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                targetRpm = NEAR_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.A)){
                targetRpm = 0;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                autoRpm = false;
                if (targetRpm == 0) {
                    targetRpm = NEAR_RPM;
                    targetHood = HOOD_MIN;
                } else if (targetRpm == NEAR_RPM) {
                    targetRpm = FAR_RPM;
                    targetHood = HOOD_MAX;
                } else {
                    targetRpm = 0;
                }
            }

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                targetHood += 0.05;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                targetHood -= 0.05;
            }
*/

            // clip our targetRpm .. do this LAST after all command processing
            if(targetRpm > MAX_RPM) targetRpm = MAX_RPM;
        }
    }
}
