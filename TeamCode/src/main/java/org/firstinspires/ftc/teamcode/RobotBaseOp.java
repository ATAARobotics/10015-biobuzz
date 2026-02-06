package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@Configurable
public abstract class RobotBaseOp extends OpMode {
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

    boolean minuteWarning = false;
    boolean endgameWarning = false;
    int loops;

    // targeting based on odometry
    double geometricTargetHeading;
    double geometricDistance;
    // offset robot / turret centers is 66.70mm
    private static double ROBOT_CENTER_TO_TURRET_INCHES = 2.626;

    public enum StartZone {NEAR, FAR}
    public enum Alliance {RED, BLUE}
    public abstract Alliance getAlliance();

    // we don't actually "know" in teleop, and also shouldn't care, so
    // we provide a default implementation
    public StartZone getStartZone() {
        return StartZone.NEAR;
    }

    public abstract boolean isAuto();

    protected abstract void bindOperatorControls();
    protected abstract void bindDriverControls();

    public boolean isRedAlliance() {
        return getAlliance() == Alliance.RED;
    }

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drive = new Drive(hardwareMap, isRedAlliance(), isAuto());
        turret = new Turret(hardwareMap, isRedAlliance(), isAuto());
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
            intake.fullPower();
            spindexer.spinModeIndex();
        }
        public void execute() {
            if (state == InState.INTAKE) {
                if (spindexer.artifactInSlot()) {
                    if (spindexer.isFull()) {
                        state = InState.SORT;
                    } else {
                        state = InState.SPIN;
                        intake.lowPower();
                        spindexer.spinIndex();
                    }
                }
            } else if (state == InState.SPIN) {
                if (spindexer.isStuck()){
                    intake.lowPower();
                }
                else {
                    intake.fullPower();
                }
                if (spindexer.atTarget()) {
                    state = InState.INTAKE;
                    intake.grab();
                    spindexer.spinModeIndex();
                }
            } else if (state == InState.SORT) {
                if (spindexer.atTarget()) {
                    int s = spindexer.currentShootSlot();
                    // if we have no green, or we're currently going
                    // to shoot a green next, we're done.
                    if (turret.pattern != -1){
                        s = s - turret.pattern;
                        if (s < 0){
                            s = s + 3;
                        }
                    }
                    if (!spindexer.haveOneGreen() || spindexer.slots[s] == Spindexer.SlotContent.Green) {
                        state = InState.DONE;
                        spindexer.spinModeIndex();
                    } else {
                        spindexer.spinIndex();
                    }
                }
            }

            // don't keep slamming balls into a stuck spindexer
         //   if (!spindexer.atTarget() && spindexer.isStuck()) {
          //      intake.stop();
         //   }
        }
        public boolean isFinished() {
            return state == InState.DONE;
        }
        public void end(boolean interrupted){
            if (spindexer.isFull()) {
                driver.gamepad.rumble(250);
                intake.stop();
            }

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
                // OR:
                // - the operator pressed A
                if ((shooter.readyToShoot() &&
                   // turret.isLocked(time) &&
                    turret.atTargetAngle()) || operator.wasJustPressed(GamepadKeys.Button.A)) {
                    state = OutState.SHOOT;
                    lastShots = shooter.getCurrentShots();
                    shotSlot = spindexer.currentSlot();
                    spindexer.spinShoot();
                }
            } else if (state == OutState.SHOOT) {
                // try just not caring about "did a shot really go up"
                // for this -- so we're just trusting the spindexer's
                // notion of how many balls
                if (spindexer.atTarget()) { //shooter.getCurrentShots() > lastShots) {
                    if (spindexer.isEmpty()) {
                        state = OutState.DONE;
                    } else {
                        state = OutState.WAIT_SHOOT;
                    }
                }
            }
            if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                state = OutState.DONE;
                turret.noLock();
                shooter.manualShootRpm();
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


    public enum SortState {SORT, DONE};
    public class SortSpindex extends CommandBase {
        private SortState state;

        public SortSpindex() {
            addRequirements(spindexer);
            addRequirements(intake);
        }
        public void initialize() {
            state = SortState.SORT;
            spindexer.spinModeIndex();
        }
        public void execute() {
            intake.lowPower();
            intake.grab();
            if (state == SortState.SORT) {
                if (spindexer.atTarget()) {
                    int s = spindexer.currentShootSlot();
                    // if we have no green, or we're currently going
                    // to shoot a green next, we're done.
                    if (turret.pattern != -1){
                        s = s - turret.pattern;
                        if (s < 0){
                            s = s + 3;
                        }
                    }
                    if (!spindexer.haveOneGreen() || spindexer.slots[s] == Spindexer.SlotContent.Green) {
                        state = SortState.DONE;
                        spindexer.spinModeIndex();
                        intake.stop();
                    } else {
                        spindexer.spinIndex();
                    }
                }
            }
        }
        public void end(boolean interrupted) {
            intake.stop();
        }
        public boolean isFinished() {
            return state == SortState.DONE;
        }
    }


    public class SoftIntake extends CommandBase {
        public SoftIntake() {
            addRequirements(intake);
        }
        public void initialize() {
            intake.lowPower();
            intake.grab();
        }
        public boolean isFinished() {
            return true;
        }
    }


    public class LookAtObelisk extends CommandBase {
        public double startTime;

        public LookAtObelisk() {
            addRequirements(turret);
        }
        public void initialize() {
            startTime = time;
        }
        public void execute() {
            double robotX = drive.getPosition().getX(DistanceUnit.INCH);
            double robotY = drive.getPosition().getY(DistanceUnit.INCH);
            double robotHeading = drive.getPosition().getHeading(AngleUnit.DEGREES);

            double turretX = robotX - (Math.cos(robotHeading) * ROBOT_CENTER_TO_TURRET_INCHES);
            double turretY = robotY - (Math.sin(robotHeading) * ROBOT_CENTER_TO_TURRET_INCHES);

            double obeliskHeading = Math.toDegrees(
                Math.atan2(141.0 - turretY, 70.5 - turretX)
                );

            // for close-zone autos, we actually look at the _side_ of
            // the obelisk and adjust .. for "blue-side" we move 5
            // degrees right and then can in theory only see the side
            // one .. for "red-size" we turn a bunch left (45?) and
            // then see the "other" side of the obelisk

            if (getStartZone() == StartZone.NEAR) {
                if (getAlliance() == Alliance.RED) {
                    // these are "field angles"
                    obeliskHeading = 90 + 45;
                } else {
                    obeliskHeading = 90 - 10;
                }
            }
            turret.faceObelisk(obeliskHeading);
        }
        public boolean isFinished(){
            // we look for up to 1 second, or until we see an Obelisk pattern
            return (time - startTime > 1.0) || (turret.pattern != -1);
        }
        public void end(boolean interrupted) {
            turret.noLock();
            // for near-zone, we will have seen the "side" of the
            // obelisk, so adjust
            if (turret.pattern != -1 ) {
                if (getStartZone() == StartZone.NEAR) {
                    if (getAlliance() == Alliance.RED) {
                        turret.pattern += 1;
                        if (turret.pattern > 2) {
                            turret.pattern = 0;
                        }
                    } else {
                        turret.pattern -= 1;
                        if (turret.pattern < 0) {
                            turret.pattern = 2;
                        }
                    }
                }
            }
        }
    }


    public class OuttakeOff extends CommandBase {
        public OuttakeOff() {
            addRequirements(shooter);
            addRequirements(turret);
        }

        public void initialize() {
            turret.noLock();
            shooter.manualShootRpm();
        }
    }


    public class PrepareToShoot extends CommandBase{
        public PrepareToShoot() {
            addRequirements(shooter);
            addRequirements(turret);
        }

        public void initialize() {
                shooter.autoShootRpm();
                turret.autoLock();
        }
        public boolean isFinished(){
            return true;
        }
    }

    public class UnShoot extends CommandBase{
        public UnShoot(){
            addRequirements(shooter);
            addRequirements(turret);
        }
        public void initialize(){
            shooter.manualShootRpm();
            turret.noLock();
        }
        public boolean isFinished(){return true;}
    }

    protected void readSensors() {
        drive.read_sensors(time);
        shooter.read_sensors(time);
        spindexer.read_sensors(time);
        turret.read_sensors(time);
        intake.read_sensors(time);
    }

    protected void readControls() {
        driver.readButtons();
        operator.readButtons();
    }

    protected void clearCache() {
        // make sure we get fresh values for all encoders
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        loops++;
    }

    protected void addTelemetry(HyperTelemetry telem) {
    }

    protected void logTelemetry() {
        //TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry); //, pack);
        telem.log("elapsed", runtime.toString());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());
        telem.log("geometric-target", geometricTargetHeading);
        telem.log("geometric-distance", geometricDistance);
        telem.log("alliance", getAlliance());
        telem.log("zone", getStartZone());

        double fps = loops / runtime.seconds();
        telem.logDrivers("average fps", fps);

        drive.addTelemetry(telem);
        shooter.addTelemetry(telem);
        turret.addTelemetry(telem);
        intake.addTelemetry(telem);
        intake.addTelemetry(telem);
        spindexer.addTelemetry(telem);
        addTelemetry(telem);

        telem.update();
    }

    @Override
    public void start() {
        runtime.reset();
        turret.reset();
        spindexer.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? 77.5 + 8.124 : 47.5 + 8.124, 8.0984, AngleUnit.DEGREES, 90));
        loops = 0;
    }

    @Override
    public void init_loop() {
        clearCache();

        // runs while the robot is "on" but we haven't pressed "play" yet
        turret.read_sensors(0.0);
      /*  telemetry.addData("Turret Servo Right", turret.getServoAngle());
        telemetry.addData("Turret Servo Left", turret.getOtherServoAngle());
        telemetry.update(); */
    }

    @Override
    public void loop() {
        clearCache();
        readControls();
        readSensors();

        if (!minuteWarning && runtime.seconds() > 60){
            minuteWarning = true;
            operator.gamepad.rumble(300);
        }
        if (!endgameWarning && runtime.seconds() > 100){
            endgameWarning = true;
            operator.gamepad.rumble(600);
        }
        // We need to rotate the FTC coordinate system 90 degrees to
        // get the pedro pathing system, and Offset by 72 inches (70.5)

        double robotHeading = drive.getPosition().getHeading(AngleUnit.DEGREES);
        double robotX = drive.getPosition().getX(DistanceUnit.INCH);
        double robotY = drive.getPosition().getY(DistanceUnit.INCH);

        // we need to offset the robot x and y values to be at the
        // center of the turret.
        double turretX = robotX - (Math.cos(robotHeading) * ROBOT_CENTER_TO_TURRET_INCHES);
        double turretY = robotY - (Math.sin(robotHeading) * ROBOT_CENTER_TO_TURRET_INCHES);

        // AAAAAAaaaaaa! okay, so FTC co-ordinate system says the
        // field is 144x144 inches. This is not true, it is actually
        // 141.5 inches. The field-center is (70.5, 70.5) NOT (72, 72)
        // if we measure from tile-edges.

        //double targetX = turret.target.fieldPosition.get(1);
        //double targetY = -turret.target.fieldPosition.get(0);
        // TODO: red vs blue targets
        double targetX = 10;
        double targetY = 135;
        if (getAlliance() == Alliance.RED){
            targetX = 141 - targetX;
        }

        double distanceA = targetX - turretX;
        double distanceB = targetY - turretY;
        geometricDistance = Math.sqrt((distanceA * distanceA) + (distanceB * distanceB));

        geometricTargetHeading = Math.toDegrees(
            Math.atan2(targetY - turretY, targetX - turretX)
        );

        turret.robotHeading = robotHeading;
        if (turret.isLocked(time)) {
            shooter.aprilDistance = turret.aprilDistance;
        }
        else{
            shooter.aprilDistance = geometricDistance;
            turret.targetHeading = geometricTargetHeading;
        }

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        logTelemetry();
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
