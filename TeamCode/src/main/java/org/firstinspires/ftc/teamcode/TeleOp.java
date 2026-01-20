package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.List;

@Configurable
public abstract class TeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;

    VoltageSensor battery;
    List<LynxModule> allHubs;

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

        // set up controls
        bindOperatorControls();
        bindDriverControls();

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
        CommandScheduler.getInstance().setDefaultCommand(shooter, shooter.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(turret, turret.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(intake, intake.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(spindexer, spindexer.new HumanInputs(operator, driver));

        // set up for bulk-reads of encoders etc (in MANUAL we *must*
        // remember to clear the cache once per cycle or we'll always
        // have stale values)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public class TriggerHeld extends Trigger {
        GamepadEx pad;
        GamepadKeys.Trigger trigger;

        public TriggerHeld(GamepadEx gp, GamepadKeys.Trigger tr) {
            pad = gp;
            trigger = tr;
        }

        @Override
        public boolean get() {
            return (pad.getTrigger(trigger) > 0.5);
        }
    }

    // state-machine to "automatically intake balls"
    // goal: fill up the spindexer
    // but: might already have 0, 1, 2 or 3 balls
    // (so _don't_ want to do anything at all if we e.g. have 3 balls)
    public enum InState {INTAKE, SPIN, SORT, DONE};
    public class AutoIntake extends CommandBase {
        private InState state;

        public AutoIntake() {
            addRequirements(spindexer);
            addRequirements(intake);
        }
        public void initialize() {
            state = InState.INTAKE;
            intake.grab();
            spindexer.spinModeIndex();
        }
        public void execute() {
            if (state == InState.INTAKE) {
                if (spindexer.artifactInSlot()) {
                    if (spindexer.isFull()) {
                        state = InState.SORT;
                    } else {
                        state = InState.SPIN;
                        intake.stop();
                        spindexer.spinIndex();
                    }
                }
            } else if (state == InState.SPIN) {
                if (spindexer.atTarget()) {
                    state = InState.INTAKE;
                    intake.grab();
                    spindexer.spinModeIndex();
                }
            } else if (state == InState.SORT) {
                // for now we just shoot green first, always .. but
                // we'll want it to be first, second or last depending
                // on the Obelisk
                if (spindexer.atTarget()) {
                    int s = spindexer.currentShootSlot();
                    // if we have no green, or we're currently going
                    // to shoot a green next, we're done.
                    if (!spindexer.haveOneGreen() || spindexer.slots[s] == Spindexer.SlotContent.Green) {
                        state = InState.DONE;
                        spindexer.spinModeIndex();
                    } else {
                        spindexer.spinIndex();
                    }
                }
            }
        }
        public boolean isFinished() {
            return state == InState.DONE;
        }
        public void end(boolean interrupted){
            // todo: rumble driver when full?
        }
    }

    // state-machine to auto-fire balls
    // goal: empty the spindexer
    // but: might already be empty!
    public enum OutState {WAIT_SHOOT, SHOOT, DONE};
    public class AutoOuttake extends CommandBase {
        private OutState state;
        private int shotSlot = -1;
        private int lastShots = -1;

        public AutoOuttake() {
            addRequirements(spindexer);
            addRequirements(shooter);
            addRequirements(turret);
        }
        public void initialize() {
            state = OutState.WAIT_SHOOT;
        }
        public void execute() {
            if (state == OutState.WAIT_SHOOT) {
                shooter.autoShootRpm();
                turret.autoLock();
                // we can shoot if:
                // - we're "at" our RPM
                // - and have April lock
                // - and Turret is at its angle
                if ((shooter.readyToShoot() &&
                   // turret.haveAprilLock &&
                    turret.atTargetAngle()) || operator.wasJustPressed(GamepadKeys.Button.A)) {
                    state = OutState.SHOOT;
                    lastShots = shooter.getCurrentShots();
                    shotSlot = spindexer.currentSlot();
                    spindexer.spinShoot();
                }
            } else if (state == OutState.SHOOT) {
                // todo: the spindexer can actually get stuck trying
                // to "go back" to its target (e.g. we overshot) but
                // .. maybe we don't care here, we should just keep
                // shooting essentially?
                // (what we actually want to do here is ask "did the shooter shoot recently")
                //if (spindexer.atTarget() && spindexer.currentSlot() != shotSlot) {
                if (shooter.getCurrentShots() > lastShots) {
                    if (spindexer.isEmpty()) {
                        state = OutState.DONE;
                    } else {
                        state = OutState.WAIT_SHOOT;
                    }
                }
            }
            if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                state = OutState.DONE;
            }
        }
        public boolean isFinished() {
            return state == OutState.DONE;
        }
        public void end(boolean interrupted){
            if (!interrupted) {
                turret.noLock();
                shooter.manualShootRpm();
            }
        }
    }
    public class ToggleShoot extends CommandBase{
        public ToggleShoot() {
            addRequirements(shooter);
            addRequirements(turret);
        }

        public void initialize() {
            if (shooter.autoRpm) {
                shooter.manualShootRpm();
                turret.noLock();
            } else {
                shooter.autoShootRpm();
                turret.autoLock();
            }
        }
        public boolean isFinished(){
            return true;
        }
    }

    private void bindDriverControls() {
        // auto intake mode
        TriggerHeld driverRight = new TriggerHeld(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        driverRight.whileActiveOnce(new AutoIntake(), true);
        // auto outtake mode
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new AutoOuttake(), true);
    }

    private void bindOperatorControls() {
        // spindexer
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            spindexer.new ShootOnce()
            );
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            new SequentialCommandGroup(
                spindexer.new ShootOnce(),
                spindexer.new ShootOnce(),
                spindexer.new ShootOnce()
                )
            );
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            spindexer.new IndexOnce()
        );
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ToggleShoot()
        );
    }

    @Override
    public void start() {
        runtime.reset();
        turret.reset();
        spindexer.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance ? 77.5 + 8.124 : 47.5 + 8.124, 8.0984, AngleUnit.DEGREES, 90));
        // this is the near-goal position inside the launch zone aligned along the outside edge of the launch line
//        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 1.191 : -1.191, 1.457, AngleUnit.DEGREES, isRedAlliance ? -45 : 45));
        loops = 0;
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play" yet
        turret.read_sensors(0.0);
        telemetry.addData("Turret Servo Angle", turret.getServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        // make sure we get fresh values for all encoders
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        loops++;
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        drive.read_sensors(time);
        shooter.read_sensors(time);
        spindexer.read_sensors(time);
        turret.read_sensors(time);
        //intake.read_sensors(time);
        double robotX = drive.getPosition().getX(DistanceUnit.INCH);
        double robotY = drive.getPosition().getY(DistanceUnit.INCH);
        //double targetX = turret.target.fieldPosition.get(1);
        //double targetY = -turret.target.fieldPosition.get(0);
        double targetX = 14.5;
        double targetY = 144 - 12.25;

        double distanceA = targetX - robotX;
        double distanceB = targetY - robotY;
        double distance = Math.sqrt((distanceA * distanceA) + (distanceB * distanceB));
        //distance = DistanceUnit.INCH.fromMeters(distance);
        turret.apriltag_heading = drive.apriltag_heading;
        turret.robot_heading = drive.getPosition().getHeading(AngleUnit.DEGREES);
        if (false && turret.haveAprilLock){
            shooter.aprilDistance = turret.april_distance;
        }
        else{
            shooter.aprilDistance = distance;
        }

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        //TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry); //, pack);
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
        telem.logDrivers("average fps", fps);
        telem.update();
//        FtcDashboard.getInstance().sendTelemetryPacket(pack);
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
