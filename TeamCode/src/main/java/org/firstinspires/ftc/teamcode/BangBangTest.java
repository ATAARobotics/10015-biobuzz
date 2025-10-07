package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BangBangTest", group="Opmode")
public class  BangBangTest extends OpMode {
    GamepadEx control;
    MotorEx motor0;
    private Servo indicatorLight;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BIG_STEP_RPM = 250;
    private static final double SMALL_STEP_RPM = 10;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;
    double BAND = 50;
    double ON_POWER = 1.0;
    double OFF_POWER = 0.0;
    boolean on = false;
    @Override
    //setting up the gamepad and motor
    public void init() {
        rpmTarget = 0;
        control = new GamepadEx(gamepad2);
        motor0 = new MotorEx(hardwareMap, "motor0");
        indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0. setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void loop() {
        double ticksPerSecond;
        ticksPerSecond = motor0.getVelocity();
        double power;
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
        if (currentRpm < rpmTarget - BAND){
            power = ON_POWER;
        }
        else {
            power = OFF_POWER;
        }
        if (rpmTarget == 0) power = 0;
        if(power < 0) power = 0;
        motor0.set(power); //when you move joystick, motor power changes
        telemetry.addData("motor0", power); //what you see on the screen
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
        if (currentRpm>4900.0 && currentRpm<5100.0) {
            indicatorLight.setPosition(0.5);
        } else {
            indicatorLight.setPosition(0.28);
        }
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        telemetry.update();
    }
}
