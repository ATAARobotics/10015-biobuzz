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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BangBangTest", group="Opmode")
public class  BangBangTest extends OpMode {
    GamepadEx control;
    private Servo indicatorLight;
    MotorGroup shooterMotor;
    MotorEx motor0;
    MotorEx motor1;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BIG_STEP_RPM = 250;
    private static final double SMALL_STEP_RPM = 10;
    double GREEN = 0.5;
    double RED = 0.28;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;
    double BAND = 1;
    double BANG_POWER = 1.0;
    double OFF_POWER = 0.0;
    double rpmTolerance = 100;
    boolean powerOn = false;
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
        shooterMotor.setRunMode(Motor.RunMode.RawPower);
        shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void loop() {
        double ticksPerSecond;
        ticksPerSecond = shooterMotor.getVelocity();
        double power;
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
        if (currentRpm < (rpmTarget - BAND) && !powerOn) {
            powerOn = true;
        }
        else if (currentRpm > (rpmTarget + BAND) && powerOn) {
            powerOn = false;
        }
        if (powerOn) power = BANG_POWER;
        else power = OFF_POWER;
        if (rpmTarget == 0) power = 0;
        if(power < 0) power = 0;
        shooterMotor.set(power); //when you move joystick, motor power changes
        telemetry.addData("power", power); //what you see on the screen
        telemetry.addData("rpmTarget", rpmTarget);
        telemetry.addData("Current RPM", currentRpm);
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
        if (Math.abs(currentRpm - rpmTarget) < rpmTolerance) {
            indicatorLight.setPosition(GREEN);
        } else {
            indicatorLight.setPosition(RED);
        }
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
        pack.put("currentRpm", currentRpm);
        pack.put("power", power);
        pack.put("motor0_ticks", motor0.motorEx.getCurrentPosition());
        pack.put("motor1_ticks", motor1.motorEx.getCurrentPosition());
        //pack.put("motor0_corrected_velocity", motor0.getCorrectedVelocity());
        //pack.put("motor1_corrected_velocity", motor1.getCorrectedVelocity());
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        telemetry.update();
    }
}
