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
    Arm arm;
    ElapsedTime runtime = new ElapsedTime();
    boolean endgameAlertTriggered = false;
    boolean parkAlertTriggered = false;

    public enum Alliance {RED, BLUE};
    public abstract Alliance getAlliance();

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        arm = new Arm(hardwareMap, getAlliance() == Alliance.RED);
        drive = new Drive(hardwareMap, driver);
        arm.drive = drive;
        drive.arm = arm;
        battery = hardwareMap.voltageSensor.get("Control Hub");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(arm);

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
        CommandScheduler.getInstance().setDefaultCommand(arm, arm.new HumanInputs(operator));
    }

    @Override
    public void start() {
        arm.reset();
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
        arm.read_sensors();

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        // Check if we're in endgame and trigger gamepad rumble for both drivers
        if (!endgameAlertTriggered && (runtime.seconds() > 90)) {
            Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 1.0, 250) // Rumble both motors 100% for 250 mSec
                    .build();
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
            Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                    .addStep(255, 255, 255, 250)  // White for 250ms
                    .build();
            gamepad1.runLedEffect(ledEffect);
            gamepad2.runLedEffect(ledEffect);
            endgameAlertTriggered = true;
        }
        if (!parkAlertTriggered && (runtime.seconds() > 110)) {
            // Create a Rumble effect that cycles through motors
            Gamepad.RumbleEffect rumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(1.0, 0.0, 500)  //  Rumble left motor 100% for 500 mSec
                    .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                    .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 250 mSec
                    .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                    .addStep(0.0, 1.0, 250)  //  Rumble right motor 100% for 250 mSec
                    .build();
            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
            // Create a LED effect that cycles through colors
            Gamepad.LedEffect ledEffect = new Gamepad.LedEffect.Builder()
                    .addStep(255, 0, 0, 250)  // Red for 250ms
                    .addStep(0, 255, 0, 250)  // Green for 250ms
                    .addStep(0, 0, 255, 250)  // Blue for 250ms
                    .build();
            gamepad1.runLedEffect(ledEffect);
            gamepad2.runLedEffect(ledEffect);
            parkAlertTriggered = true;
        }

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        arm.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // note, seems that "drawing stuff" commands have to go in their own packet

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
                 .addData("Arm Extension", arm.target_extension)
                 .addData("Arm Claw Position", arm.claw_servo.getPosition())
                 .addData("Arm Wrist Position", arm.wrist_servo.getPosition());

        telemetry.update();
    }

    @Override public void stop() {
        drive.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }

}
