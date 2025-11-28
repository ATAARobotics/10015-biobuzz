package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Shooter extends SubsystemBase {
    private final Servo indicatorLight;
    MotorEx motor0;
    MotorEx motor1;
    MotorGroup shooterMotor;

    double ticksPerSecond;
    double power;
    double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)
    boolean readyToCount = false;
    int shotsFired = 0;
    public static double HIGH_STATE_OFFSET = -250;
    public static double LOW_STATE_OFFSET = -500;
    //Tuned on November 27:
    public static double FAR_RPM = 4950;
    public static double NEAR_RPM = 4500;
    double RED = 0.28;
    double GREEN = 0.5;
    double BAND = 10; //not tested
    double BANG_POWER = 1.0;
    public static double RPM_TOLERANCE = 250;
    boolean powerOn = false;
    private static final double TICKS_PER_REV = 28.0;  // fixme: get from motor
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double targetRpm;
    double currentRpm;
    double voltage; // current battery voltage

    public static double kv = 0.0021; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    public Shooter(HardwareMap hardwareMap) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "shooter0", Motor.GoBILDA.BARE);
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor0.setInverted(true);

        motor1 = new MotorEx(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(false);

        shooterMotor = new MotorGroup(motor0, motor1);
        targetRpm = 0;
        indicatorLight = hardwareMap.get(Servo.class, "indicator");
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?
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
        ticksPerSecond = motor0.getVelocity();
        currentRpm = (ticksPerSecond * 60) / TICKS_PER_REV;
        voltage = battery.getVoltage();
    }
    public class Shoot extends CommandBase {
        private Spindexer spinner;
        private Intake takeIn;
        private boolean didShoot;
        private int shots;

        public Shoot(Spindexer s, Intake i) {
            spinner = s;
            takeIn = i;
            didShoot = false;
        }

        @Override
        public void initialize() {
            targetRpm = FAR_RPM;
            shots = shotsFired;
            didShoot = false;
        }

        @Override
        public void execute() {
            if (readyToShoot() && ! didShoot) {
                spinner.spinccw();
                didShoot = true;
            }
            if (didShoot) {
                takeIn.grab();
            }
        }

        @Override
        public boolean isFinished() {
            return shotsFired > shots;
        }
        @Override
        public void end(boolean interrupted){
            targetRpm = 0;
            takeIn.stop();
        }
    }

    public CommandBase shoot(Spindexer s, Intake i){
        return new Shoot(s,i);
    }

    public boolean readyToShoot() {
        return (targetRpm > 0 && Math.abs(currentRpm - targetRpm) < RPM_TOLERANCE);
    }

    @Override
    public void periodic() {
        appliedVoltage = (kv * targetRpm) + ks;
        power = appliedVoltage / voltage;
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

        shooterMotor.set(power);

        // indicator lights
        if (targetRpm > 0) {
            if (readyToShoot()) {
                indicatorLight.setPosition(GREEN);
            } else {
                indicatorLight.setPosition(RED);
            }
        } else {
            // turn off the light if we're not spinning
            indicatorLight.setPosition(0);
        }

        // count shots
        if (currentRpm > targetRpm + HIGH_STATE_OFFSET) {
            readyToCount = true;
        }
        if (readyToCount && currentRpm < targetRpm + LOW_STATE_OFFSET){
            shotsFired += 1;
            readyToCount = false;
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
            */
            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                if (targetRpm == 0) {
                    targetRpm = NEAR_RPM;
                } else if (targetRpm == NEAR_RPM) {
                    targetRpm = FAR_RPM;
                } else {
                    targetRpm = 0;
                }
            }

            // clip our targetRpm .. do this LAST after all command processing
            if(targetRpm > MAX_RPM) targetRpm = MAX_RPM;
        }
    }
}
