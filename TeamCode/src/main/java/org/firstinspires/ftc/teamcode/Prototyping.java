package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Prototyping", group="Opmode")
public class Prototyping extends OpMode {
    GamepadEx control;
    Motor motor0;

    @Override
    //setting up the gamepad and motor
    public void init() {
        control = new GamepadEx(gamepad1);
        motor0 = new Motor(hardwareMap, "motor0");
    }

    @Override
    public void loop() {
        double power = control.getLeftY(); //'getleftY' means up and down on left joystick

        motor0.set(power); //when you move joystick, motor power changes


        telemetry.addData("motor0", power); //what you see on the screen
        telemetry.update();
    }
}
