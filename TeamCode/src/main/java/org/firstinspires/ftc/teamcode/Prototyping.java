package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Prototyping", group="Opmode")
public class  Prototyping extends OpMode {
    GamepadEx control;
    MotorGroup shooterMotor;
    MotorEx motor0;
    MotorEx motor1;
    private Servo indicatorLight;
    double RED = 0.28;
    double GREEN = 0.5;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BIG_STEP_RPM = 250;
    private static final double SMALL_STEP_RPM = 10;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;

    private boolean on = false;
    PIDController velocity;
    public static double velocity_p = 0.01;
    public static double velocity_i = 0.0;
    public static double velocity_d = 0.0004;
    public static double kv = 0.0021; //kv is Feed Forward Model slope, determined experimentally with flywheel
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)


    @Override
    //setting up the gamepad and motor
    public void init() {
        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor0.setInverted(false);

        motor1 = new MotorEx(hardwareMap, "motor1");
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(true);

        shooterMotor = new MotorGroup(motor0, motor1);
        rpmTarget = 0;
        control = new GamepadEx(gamepad2);
        indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");
        velocity = new PIDController(velocity_p, velocity_i, velocity_d);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void loop() {
        velocity.setP(velocity_p);
        velocity.setI(velocity_i);
        velocity.setD(velocity_d);
        velocity.setSetPoint(rpmTarget);
        double ticksPerSecond;
        ticksPerSecond = shooterMotor.getVelocity();
        double power;
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
        double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)
        appliedVoltage = kv*rpmTarget+ks;
        power = appliedVoltage/battery.getVoltage();
        power += velocity.calculate(currentRpm); //Change power to += when Feed Forward is used

    power = 1.0;

        if (rpmTarget == 0) power = 0;
        if(power < 0) power = 0;
        shooterMotor.set(power);
        telemetry.addData("motor0", power); //what you see on the screen
        telemetry.addData("rpmTarget", rpmTarget);
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Applied Voltage", appliedVoltage);
        telemetry.addData("Battery Voltage", battery.getVoltage());
        control.readButtons();

        if (control.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            rpmTarget += BIG_STEP_RPM;
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            rpmTarget -= BIG_STEP_RPM;
            if(rpmTarget < 0) rpmTarget = 0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.Y)){
            rpmTarget += SMALL_STEP_RPM;
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.A)) {
            rpmTarget -= SMALL_STEP_RPM;
            if (rpmTarget < 0) rpmTarget = 0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.X)){
            rpmTarget = 0;
        }
        if (currentRpm>4900.0 && currentRpm<5100.0) {
            indicatorLight.setPosition(GREEN);
        } else {
            indicatorLight.setPosition(RED);
        }
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("currentRpm", currentRpm);
        pack.put("power", power);
        pack.put("motor0_ticks", motor0.getCurrentPosition());
        pack.put("motor1_ticks", motor1.getCurrentPosition());
        //pack.put("motor0_corrected_velocity", motor0.getCorrectedVelocity());
        //pack.put("motor1_corrected_velocity", motor1.getCorrectedVelocity());
        pack.put("velocity_p:", velocity_p);
        pack.put("velocity_i:", velocity_i);
        pack.put("velocity_d:", velocity_d);
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        telemetry.update();
    }
}
