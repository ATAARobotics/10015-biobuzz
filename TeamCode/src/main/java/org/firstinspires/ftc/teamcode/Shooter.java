package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    //MotorGroup shooterMotor;
    //private Servo indicatorLight;
    private Servo feeder;

    private Servo indicatorLight;
    MotorEx motor0;
    //MotorEx motor1;
    double ticksPerSecond;
    double power;
    double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)
    double RED = 0.28;
    double GREEN = 0.5;
    double BAND = 10; //not tested
    double BANG_POWER = 1.0;
    double rpmTolerance = 100;
    boolean powerOn = false;
    private static final double FAR_RPM = 4700;
    private static final double NEAR_RPM = 3500;
    private static final double TICKS_PER_REV = 28.0;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;
    double idlePos = 0.9;

    public enum LaunchState {IDLE, SHOOT};

    public LaunchState launchState;

    //PIDController velocity;
    public static double kv = 0.0021; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    public Shooter(HardwareMap hardwareMap, GamepadEx operator) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor0.setInverted(true);

        /*motor1 = new MotorEx(hardwareMap, "motor1");
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(false);

        shooterMotor = new MotorGroup(motor0, motor1); */
        feeder = hardwareMap.get(Servo.class, "feeder");

 //       shooterMotor = new MotorGroup(motor0, motor1);
        rpmTarget = 0;
        //indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?
    }

    public void read_sensors(double time) {
        // get any inputs from our encoders or other sensors
        ticksPerSecond = motor0.getVelocity();
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
    }

    public void init() {
        launchState = LaunchState.IDLE;
    }

    public void loop(GamepadEx control) {
        // decide what to do based on sensors and human inputs from controller
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

        if (control.wasJustPressed(GamepadKeys.Button.B)){
            rpmTarget = FAR_RPM;
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.X)){
            rpmTarget = NEAR_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.A)){
            rpmTarget = 0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && control.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            feed();
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
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
        pack.put("Feeder Idle Position", idlePos);
    }
    void feed() {
        if (launchState == LaunchState.IDLE && currentRpm > rpmTarget && rpmTarget > 0) {
            launchState = LaunchState.SHOOT;
            feeder.setPosition(1);
        } else if (launchState == LaunchState.SHOOT) {
            launchState = LaunchState.IDLE;
            feeder.setPosition(idlePos);
        }
    }
}
