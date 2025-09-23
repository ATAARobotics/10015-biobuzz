package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Prototyping", group="Opmode")
public class  Prototyping extends OpMode {
    GamepadEx control;
    MotorEx motor0;

    PIDController velocity;
    public static double velocity_p;

    @Override
    //setting up the gamepad and motor
    public void init() {
        control = new GamepadEx(gamepad1);
        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0. setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        velocity = new PIDController(velocity_p, 0, 0);
        velocity.setSetPoint(2000);
    }

    @Override
    public void loop() {
        velocity.setP(velocity_p);
        double ticksPerSecond;
        ticksPerSecond = motor0.getVelocity();
        double power = control.getLeftY(); //'getleftY' means up and down on left joystick
    power = velocity.calculate(ticksPerSecond);
        motor0.set(power); //when you move joystick, motor power changes
    telemetry.addData("ticksPerSecond", ticksPerSecond);
       telemetry.addData("motor0", power); //what you see on the screen
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }
}
