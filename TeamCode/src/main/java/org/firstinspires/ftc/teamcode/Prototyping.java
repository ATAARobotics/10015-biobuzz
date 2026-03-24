package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Prototyping", group="Opmode")
public class  Prototyping extends OpMode {
    GamepadEx control;
    Shooter shooter;

    @Override
    //setting up the gamepad and motor
    public void init() {
        shooter = new Shooter(hardwareMap);
        control = new GamepadEx(gamepad2);
        CommandScheduler.getInstance().registerSubsystem(shooter);
    }

    @Override
    public void loop() {
        control.readButtons();
        shooter.read_sensors(time);

        if (control.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            shooter.MANUAL_RPM += 200;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            shooter.MANUAL_RPM -= 200;
            if (shooter.MANUAL_RPM < 0) shooter.MANUAL_RPM = 0.0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.Y)){
            shooter.MANUAL_RPM = 0.0;
            shooter.manualShootRpm();
        }
        if (control.wasJustPressed(GamepadKeys.Button.A)) {
        }
        if (control.wasJustPressed(GamepadKeys.Button.X)){
        }

        CommandScheduler.getInstance().run();


        HyperTelemetry telem = new HyperTelemetry(telemetry);
        shooter.addTelemetry(telem);
        telemetry.update();
    }
}
