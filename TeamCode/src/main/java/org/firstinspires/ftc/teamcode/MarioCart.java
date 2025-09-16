package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Mario Kart", group="Opmode")
public class MarioCart extends OpMode {
    GamepadEx controller;
    Drive drive;
    Arm arm;
    VoltageSensor battery;

    // high chamber "RIGHT" start
    //public static double X = 0.43;
    //public static double Y = -1.61;
    // high basket ("away from red line") start
    public static double X = -0.76;
    public static double Y = -1.612;
    public static double HEADING = 0.0;

    public static double ARM_ANGLE = 0.0;
    public static double ARM_EXTENSION = 0.0;
    public static double WRIST_POSITION = 0.5;
    public static double PALM_ANGLE = Arm.PALM_MIDDLE;
    public static boolean CLAW_OPEN = true;

    // notes:
    // start-position: left-edge of robot lined up to left-most tile
    // measures at 88.3cm + center-offset (15.4mm)
    // so 103.7

    @Override
    public void init() {
        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        arm = new Arm(hardwareMap, true);
        drive = new Drive(hardwareMap, null);
        arm.drive = drive;
        drive.arm = arm;
        battery = hardwareMap.voltageSensor.get("Control Hub");
        controller = new GamepadEx(gamepad1);

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

    public Command doNothing(long timeout) {
        return new CommandBase() {}.withTimeout(timeout);
    }

    // origin is center of the field, in meters
    // these measurements seem to be to "the OTOS" center
    @Override
    public void start() {
        drive.setPosition(new SparkFunOTOS.Pose2D(X, Y, 0));
    }

    @Override
    public void loop() {
        controller.readButtons();
        drive.read_sensors(time);
        arm.read_sensors();


        if (controller.wasJustPressed(GamepadKeys.Button.X)) {
            CommandScheduler.getInstance().schedule(
                drive.moveCarefully(X, Y, HEADING).withTimeout(5000)
                );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            CommandScheduler.getInstance().schedule(
                drive.moveQuickly(X, Y, HEADING).withTimeout(5000)
                );
        }

        if (controller.wasJustPressed(GamepadKeys.Button.B)) {
            CommandScheduler.getInstance().schedule(
                    arm.moveTo(ARM_ANGLE, ARM_EXTENSION,WRIST_POSITION, PALM_ANGLE, CLAW_OPEN? Arm.CLAW_OPEN : Arm.CLAW_CLOSED).withTimeout(5000)
            );
        }

        // build up your auto sequences here
        if (controller.wasJustPressed(GamepadKeys.Button.Y)) {
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    // make sure claw closed
                    arm.moveTo(0, 0, 1.0, 0.5, Arm.CLAW_CLOSED).withTimeout(200)
                )
            );
        }


        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        drive.add_telemetry(pack);
        arm.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().h);
        editor.apply();
        drive.stop();
    }
}
