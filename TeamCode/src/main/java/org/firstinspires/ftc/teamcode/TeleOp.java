package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public abstract class TeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    VoltageSensor battery;
    Drive drive;
    Shooter shooter;
    Intake intake;
    Turret turret;
    Spindexer spindexer;
    ElapsedTime  runtime = new ElapsedTime();
    int loops;

    public enum Alliance {RED, BLUE}
    public abstract Alliance getAlliance();
    boolean isRedAlliance;

    @Override
    public void init() {
        isRedAlliance = getAlliance() == Alliance.RED;

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drive = new Drive(hardwareMap, isRedAlliance);
        turret = new Turret(hardwareMap, isRedAlliance);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        //telemetry.addData("Pinpoint Firmware Version", drive.pinpoint.getDeviceVersion());
        //telemetry.update();

        battery = hardwareMap.voltageSensor.get("Control Hub");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(spindexer);

        // set up Operator controls
        bindOperatorControls();

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
        CommandScheduler.getInstance().setDefaultCommand(shooter, shooter.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(turret, turret.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(intake, intake.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(spindexer, spindexer.new HumanInputs(operator, driver));
    }

    private void bindOperatorControls() {
        // spindexer
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            new SequentialCommandGroup(
                shooter.new NearZone(),
                spindexer.new ShootOnce(),
                shooter.new WaitForShot()
                )
            );
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            spindexer.new IndexOnce()
            );
    }

    @Override
    public void start() {
        runtime.reset();
        turret.reset();
        spindexer.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 0.281 : -0.281, -1.552, AngleUnit.DEGREES, 0));
        // this is the near-goal position inside the launch zone aligned along the outside edge of the launch line
//        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 1.191 : -1.191, 1.457, AngleUnit.DEGREES, isRedAlliance ? -45 : 45));
        loops = 0;
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play" yet
        telemetry.addData("Turret Servo Angle", turret.getServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        loops++;
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        drive.read_sensors(time);
        shooter.read_sensors(time);
        spindexer.read_sensors(time);
        //turret.read_sensors(time);
        //intake.read_sensors(time);
        turret.apriltag_heading = drive.apriltag_heading;
        turret.robot_heading = drive.getPosition().getHeading(AngleUnit.DEGREES);

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry, pack);
        telem.log("elapsed", runtime.toString());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());

        drive.addTelemetry(telem);
        shooter.addTelemetry(telem);
        turret.addTelemetry(telem);
        intake.addTelemetry(telem);
        spindexer.addTelemetry(telem);

        // log some drivetrain information always too
        Pose2D drivePosition = drive.getPosition();
        double x = drivePosition.getX(DistanceUnit.METER);
        double y = drivePosition.getY(DistanceUnit.METER);
        double h = drivePosition.getHeading(AngleUnit.DEGREES);
        telem.log("position-x", x);
        telem.log("position-y", y);
        telem.log("position-heading", h);
        telem.logDrivers("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", x, y, h);
        double fps = loops / runtime.seconds();
        telem.logDrivers("frames per second", fps);
        telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        drive.stop();
        turret.stop();
        shooter.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
