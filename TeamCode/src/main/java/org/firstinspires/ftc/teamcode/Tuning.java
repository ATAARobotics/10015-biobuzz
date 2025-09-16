package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Tuning")
public class Tuning extends OpMode {
    GamepadEx controller;
    Drive drive;
    Arm arm;

    @Override
    public void init() {
        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        controller = new GamepadEx(gamepad1);
        arm = new Arm(hardwareMap, true);
        drive = new Drive(hardwareMap, null);
        arm.drive = drive;
        drive.arm = arm;

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(arm);

        arm.reset();
        drive.reset();
    }

    @Override
    public void init_loop() {
        arm.read_sensors();
        arm.periodic();
        // maaaaybe could do this, but ... dangerous?
        //CommandScheduler.getInstance().run();
    }

    // origin is center of the field, in meters
    // these measurements seem to be to "the OTOS" center
    @Override
    public void start() {
        //drive.setPosition(new SparkFunOTOS.Pose2D(0.43, -1.61, 0));
        drive.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

// NOTES
// possibly better PID / etc values
//
// distance_tolerance 0.001
// static_f_forward 0.20
// static_f_sensitive 0.0
// static_f_strafe 0.25
// p=2.0, i=0.0, d=0.2

    @Override
    public void loop() {
        controller.readButtons();
        drive.read_sensors(time);
        arm.read_sensors();

        if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            CommandScheduler.getInstance().schedule(
                drive.moveCarefully(0.0, 0.6096, 0)
                //drive.moveCarefully(0.0, 1.0, 0)
                );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            CommandScheduler.getInstance().schedule(
                drive.moveCarefully(0.0, 0.0, 0)
                );
        }

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        drive.add_telemetry(pack);
        arm.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
