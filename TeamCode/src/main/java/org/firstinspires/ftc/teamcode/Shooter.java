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
    public static double HIGH_STATE = 3300;
    public static double LOW_STATE = 3000;
    double RED = 0.28;
    double GREEN = 0.5;
    double BAND = 10; //not tested
    double BANG_POWER = 1.0;
    double rpmTolerance = 50;
    boolean powerOn = false;
    private static final double FAR_RPM = 4900;
    private static final double NEAR_RPM = 3500;
    private static final double TICKS_PER_REV = 28.0;  // fixme: get from motor
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;
    double voltage; // current battery voltage

    public static double kv = 0.0021; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    public Shooter(HardwareMap hardwareMap) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        motor0.setInverted(true);

        motor1 = new MotorEx(hardwareMap, "motor1");
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(false);

        shooterMotor = new MotorGroup(motor0, motor1);
        rpmTarget = 0;
        indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?
    }

    public void reset() {
        stop();
    }

    public void stop() {
        rpmTarget = 0;
        motor0.set(0);
        motor1.set(0);
    }
    public void read_sensors(double time) {
        // get any inputs from our encoders or other sensors
        ticksPerSecond = motor0.getVelocity();
        currentRpm = (ticksPerSecond * 60) / TICKS_PER_REV;
        voltage = battery.getVoltage();
    }

    public boolean readyToShoot() {
        return (rpmTarget > 0 && Math.abs(currentRpm - rpmTarget) < rpmTolerance);
    }

    @Override
    public void periodic() {
        appliedVoltage = (kv * rpmTarget) + ks;
        power = appliedVoltage / voltage;
        //power += velocity.calculate(currentRpm); //Change power to += when Feed Forward is used
        if (currentRpm < (rpmTarget - BAND) && !powerOn) {
            powerOn = true;
        }
        else if (currentRpm > (rpmTarget + BAND) && powerOn) {
            powerOn = false;
        }
        if (powerOn) power = BANG_POWER;
        if (rpmTarget == 0) power = 0;
        if (power < 0) power = 0;

        // indicator lights
        if (rpmTarget > 0) {
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
        if (currentRpm > HIGH_STATE) {
            readyToCount = true;
        }
        if (readyToCount && currentRpm < LOW_STATE){
            shotsFired += 1;
            readyToCount = false;
        }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.logBoth("motor0", power); //what you see on the screen
        telem.logBoth("rpmTarget", rpmTarget);
        telem.logBoth("Current RPM", currentRpm);
        telem.logBoth("Applied Voltage", appliedVoltage);
        telem.logBoth("Shots Fired" , shotsFired);
       // pack.put("ticksPerSecond", ticksPerSecond);
        telem.log("shooter-rpm-target", rpmTarget);
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

            if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                rpmTarget = NEAR_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.A)){
                rpmTarget = 0;
            }

            // clip our rpmTarget .. do this LAST after all command processing
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
    }
}
