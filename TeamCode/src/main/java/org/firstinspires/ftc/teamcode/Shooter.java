package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter extends SubsystemBase {
    //MotorGroup shooterMotor;
    //private Servo indicatorLight;
    private Servo feeder;

    private Servo indicatorLight;
    MotorEx motor0;
    //MotorEx motor1;
    double ticksPerSecond;
    double power;
    double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)
    boolean readyToCount = false;
    int shotsFired = 0;
    public static double HIGH_STATE = 3300;
    public static double LOW_STATE = 2900;
    double RED = 0.28;
    double GREEN = 0.5;
    double BAND = 10; //not tested
    double BANG_POWER = 1.0;
    double rpmTolerance = 100;
    boolean powerOn = false;
    private static final double FAR_RPM = 4900;
    private static final double NEAR_RPM = 3500;
    double STEP_RPM = 200;
    private static final double TICKS_PER_REV = 28.0;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;
    public static double FEEDER_LIMIT = 0.58;
    public static double FEEDER_CLOSED = 0.6;
    public static double FEEDER_OPEN = 1.0;
    public static double FEED_TIME = 0.20; //The feeder servo runs this long in seconds when a shot is requested.

    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    public static double TIME_BETWEEN_SHOTS = 1.2;

    /*
     * Here we create two timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();


    public enum LaunchState {IDLE, FEED, SHOOT};

    private LaunchState launchState;

    //PIDController velocity;
    public static double kv = 0.0021; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    public Shooter(HardwareMap hardwareMap) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        motor0.setInverted(true);

        /*motor1 = new MotorEx(hardwareMap, "motor1");
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(false);

        shooterMotor = new MotorGroup(motor0, motor1); */
        feeder = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(FEEDER_OPEN);
        launchState = LaunchState.IDLE;

 //       shooterMotor = new MotorGroup(motor0, motor1);
        rpmTarget = 0;
        //indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?
    }

    public void reset() {
        feeder.setPosition(FEEDER_OPEN);
        launchState = LaunchState.IDLE;
    }

    public void stop() {
        rpmTarget = 0;
        motor0.set(0);
    }
    public void read_sensors(double time) {
        // get any inputs from our encoders or other sensors
        ticksPerSecond = motor0.getVelocity();
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
    }

    @Override
    public void periodic() {
        appliedVoltage = kv*rpmTarget+ks;
        power = appliedVoltage/battery.getVoltage();
        //power += velocity.calculate(currentRpm); //Change power to += when Feed Forward is used
        if (currentRpm < (rpmTarget - BAND) && !powerOn) {
            powerOn = true;
        }
        else if (currentRpm > (rpmTarget + BAND) && powerOn) {
            powerOn = false;
        }
        if (powerOn) power = BANG_POWER;
        if (rpmTarget == 0) power = 0;
        if(power < 0) power = 0;
        motor0.set(power); //when you move joystick, motor power changes

        switch (launchState) {
            case IDLE:
                break;
            case FEED:
                if (rpmTarget > 0 && Math.abs(currentRpm - rpmTarget) < rpmTolerance) {
                    if (FEEDER_CLOSED < FEEDER_LIMIT) FEEDER_CLOSED = FEEDER_LIMIT;
                    feeder.setPosition(FEEDER_CLOSED);
                    feederTimer.reset();
                    launchState = LaunchState.SHOOT;
                }
                break;
            case SHOOT:
                if (feederTimer.seconds() > FEED_TIME) {
                    feeder.setPosition(FEEDER_OPEN);
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS)
                        launchState = LaunchState.IDLE; // ball has been successfully launched
                }
                break;
        }
        if (currentRpm > HIGH_STATE) {
            readyToCount = true;
        }
        if (readyToCount && currentRpm < LOW_STATE){
            shotsFired += 1;
             readyToCount = false;
        }
        if (shotsFired == 3){
          //  launchState = LaunchState.IDLE;
            rpmTarget = 0;
           // shotsFired = 0;
        }
        //if (Math.abs(currentRpm - rpmTarget) < rpmTolerance) {
           // indicatorLight.setPosition(GREEN);
       // } else {
        //    indicatorLight.setPosition(RED);
        //}
    }

    public void add_telemetry(TelemetryPacket pack, Telemetry telemetry) {
        telemetry.addData("motor0", power); //what you see on the screen
        telemetry.addData("rpmTarget", rpmTarget);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Applied Voltage", appliedVoltage);
        telemetry.addData("Launch State", launchState.toString());
        telemetry.addData("Shots Fired" , shotsFired);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
    }

    public Command shoot(int shotsToFire) {
        return new Shoot(shotsToFire);
    }
    public class Shoot extends CommandBase {
        int targetShots;
        public Shoot(int shotsToFire) {
            addRequirements(Shooter.this);
            this.targetShots = shotsFired + shotsToFire;
        }

        @Override
        public void initialize() {
            rpmTarget = NEAR_RPM;
        }

        @Override
        public void execute() {
            if (launchState == LaunchState.IDLE) {
                if (targetShots > 0) {
                    launchState = LaunchState.FEED;
                    shotTimer.reset();
                }
            }
        }

        @Override
        public void end(boolean interrupted) {
            stop();
        }

        @Override
        public boolean isFinished() {
            if (shotsFired >= targetShots){
                return (true);
            }
            return (false);
        }
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
            if (driver.wasJustPressed(GamepadKeys.Button.B) || operator.wasJustPressed(GamepadKeys.Button.B)){
                rpmTarget = FAR_RPM;
                if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.X) || operator.wasJustPressed(GamepadKeys.Button.X)){
                rpmTarget = NEAR_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.A) || operator.wasJustPressed(GamepadKeys.Button.A)){
                rpmTarget = 0;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                rpmTarget += STEP_RPM;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                rpmTarget -= STEP_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && launchState == LaunchState.IDLE) {
                // the user would like to fire a new shot
                launchState = LaunchState.FEED;
                shotTimer.reset();
            }
        }
    }
}
