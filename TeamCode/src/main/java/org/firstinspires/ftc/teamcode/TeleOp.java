package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public abstract class TeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    VoltageSensor battery;
    Drive drive;
    ElapsedTime runtime = new ElapsedTime();

    public enum Alliance {RED, BLUE};
    public abstract Alliance getAlliance();

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drive = new Drive(hardwareMap, driver);
        battery = hardwareMap.voltageSensor.get("Control Hub");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
    }

    @Override
    public void start() {
        drive.reset();
        runtime.reset();
        // set starting position
        //drive.setPosition(new SparkFunOTOS.Pose2D(-1.03,-1.61,0));
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play" yet
    }

    @Override
    public void loop() {
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        drive.read_sensors(time);

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // Send telemetry messages to explain controls and show robot status
        // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
        SparkFunOTOS.Pose2D drivePosition = drive.getPosition();

        // FIXME TODO put into FTC Dashboard too, for most of this
        telemetry.addData("Drive/Strafe", "Right Stick")
                 .addData("Turn", "Left Stick")
                 .addData("Wrist Up/Middle/Down", "Dpad Up & Down")
                 .addData("Claw Open/Closed", "X Button")
                 .addData("-", "-------")
                 .addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.x, drivePosition.y, drivePosition.h)
            ;

        telemetry.update();
    }

    @Override public void stop() {
        drive.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
