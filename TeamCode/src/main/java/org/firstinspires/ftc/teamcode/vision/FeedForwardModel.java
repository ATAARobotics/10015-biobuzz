package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FeedForwardModel", group="Opmode")
public class  FeedForwardModel extends OpMode {
    GamepadEx control;
    MotorEx motor0;
    VoltageSensor battery;

    private static final double TICKS_PER_REV = 28.0;
    private static final double STEP_POWER= 0.1;
    double power = 0.0;

    double MAX_POWER = 1.0;
    double currentRpm;
    @Override
    //setting up the gamepad and motor
    public void init() {
        control = new GamepadEx(gamepad2);
        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0. setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void loop() {
        double ticksPerSecond;
        ticksPerSecond = motor0.getVelocity();
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;

        control.readButtons();
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
             power += STEP_POWER;
            if(power > MAX_POWER) power = MAX_POWER;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            power -= STEP_POWER;
            if(power < 0) power = 0;
        }

        double appliedVoltage = battery.getVoltage() * power;
        motor0.set(power); //when you move joystick, motor power changes

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
        pack.put("Applied Voltage", appliedVoltage);
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        telemetry.addData("motor0", power); //what you see on the screen
        telemetry.addData("Current RPM", currentRpm);
        telemetry.addData("Applied Voltage", appliedVoltage);
        telemetry.addData("Battery Voltage", battery.getVoltage());
        telemetry.update();
    }
}
