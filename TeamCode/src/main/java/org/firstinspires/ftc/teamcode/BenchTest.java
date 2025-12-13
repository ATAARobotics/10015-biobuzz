package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BenchTest", group="Opmode")
public class BenchTest extends OpMode {
    Servo hood;
    //public static double HOOD_MAX = 0.75;
   // public static double HOOD_MIN = 0.35;

    double targetHood;
    double STEP_HOOD = 0.05;
    double HOOD_EXTENDED = 1.0;
    GamepadEx control;

    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        control = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        control.readButtons();

        if (control.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            targetHood += STEP_HOOD;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            targetHood -= STEP_HOOD;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            targetHood = HOOD_EXTENDED;
        }


        hood.setPosition(targetHood);

        telemetry.addData("hood target", targetHood);
        telemetry.update();
    }
}




