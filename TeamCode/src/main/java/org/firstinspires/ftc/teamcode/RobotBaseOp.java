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

import java.util.LinkedList;
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
    LinkedList<Pose2D> recentPositions;

    boolean minuteWarning = false;
    boolean endgameWarning = false;
    int loops;

    // targeting based on odometry
    double geometricTargetHeading;
    double aimOffsetX;
    double aimOffsetY;
    double predictedX;
    double predictedY;
    double geometricDistance;
    // offset robot / turret centers is 66.70mm
    private static double ROBOT_CENTER_TO_TURRET_INCHES = 2.626;
    public static double GEOM_TARGET_X = 5;
    public static double GEOM_TARGET_Y = 138;
    public int operatorPattern = 0;

    public static double FAR_TARGET_X_BLUE = 9.5;
    public static double FAR_TARGET_X_RED =  3.5;
    public static double SHOOT_PREDICT = 0.570;

    public static double SHOOT_PAUSE_WAIT = 0.120;

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
        turret = new Turret(hardwareMap, isRedAlliance(), isAuto(), drive.turretEncoder);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        battery = hardwareMap.voltageSensor.get("Control Hub");

        recentPositions = new LinkedList<Pose2D>();

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
    //
    // for v3 robot:
    // - run intake
    // - await front + back beambreaks broken
    //   - spindex once
    //   - await front beambreak broken
    //   - full
    // (note: we no longer sort here, that's a special mode, since
    // it's kind of jam-prone currently and we don't need to during
    // most teleop)
    public enum InState {FIRST_TWO, SPIN, THIRD, WAIT_PIN, DONE};
    public class AutoIntake extends CommandBase {
        private InState state;
	private double startPinWait = 0.0;

        public AutoIntake() {
            addRequirements(spindexer);
            addRequirements(intake);
        }
        public void initialize() {
	    startPinWait = time;  // redundant
            // if we're already full, do not intake
            if (spindexer.isFull()) {
		// note: theoretically we're "done" here, but if the
		// driver is asking for intake we try regardless...
		spindexer.unPinBalls();
                state = InState.THIRD;
            } else {
                // driver often "pulses" the intake trigger .. so if
                // we've just before this gotten to "SPIN" or past
                // state, and the intake gets pulsed, we need to
                // actually start in THIRD.
                //
                // we can know this if the "front" slot is empty (or
                // maybe similarly if the other two slots are full)
                if (spindexer.artifactCount() == 2/* && !spindexer.artifactInSlot()*/) {
                    state = InState.THIRD;
                } else {
                    state = InState.FIRST_TWO;
                }
	    }

	    // no matter what, we DO want to run the inteake because
	    // the driver said so
	    intake.grab();
	    intake.fullPower();
	    spindexer.spinModeIndex();
        }
        public void execute() {
            if (state == InState.FIRST_TWO) {
		// waiting for the first TWO slots to be full (the
		// front slot will be full briefly or longer as the
		// first ball goes through (or settles there) but we
		// need both to be there
                if (spindexer.atTarget() && spindexer.haveFrontAndBack()) {
                    state = InState.SPIN;
                    intake.lowPower();
                    spindexer.spinIndex();
                }
            } else if (state == InState.SPIN) {
		// we are spinning one time to put the two balls we
		// have at the back
                if (spindexer.atTarget()) {
                    state = InState.THIRD;
                    intake.grab();
                    intake.fullPower();
                    // why do we need to set the mode?
                    ///spindexer.spinModeIndex();
                }
            } else if (state == InState.THIRD) {
		// awaiting our third ball
                if (spindexer.atTarget() && spindexer.haveArtifactFront()) {
		    startPinWait = time;
                    state = InState.WAIT_PIN;
                }
            } else if (state == InState.WAIT_PIN) {
		// wait some time before pinning, so we don't
		// accidentally squirt ball out the second the
		// beambreak is broken.
		double elapsed = time - startPinWait;
		if (elapsed > spindexer.PIN_WAIT_MS) {
		    spindexer.pinBalls();
		    state = InState.DONE;
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
                if (true){//!isAuto()) {
                    intake.stop();
                }
            }

        }
    }
    public class OperatorPattern extends CommandBase{
        @Override
        public void initialize() {
            operatorPattern += 1;
            if (operatorPattern > 2){
                operatorPattern = 0;
            }
        }
        public boolean isFinished(){
            return true;
        }
    }

    // state-machine to auto-fire balls
    // goal: empty the spindexer
    // but: might already be empty!
    public enum OutState {WAIT_SHOOT, SHOOT, PAUSE, DONE};
    public class AutoOuttake extends CommandBase {
        private OutState state;
        private int shotSlot = -1;
        private int lastShots = -1;
	private double startPause = 0.0;

        public AutoOuttake() {
            addRequirements(spindexer);
            addRequirements(shooter);
            addRequirements(turret);
            addRequirements(intake);
        }
        public void initialize() {
            state = OutState.WAIT_SHOOT;
            intake.grab();
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
                     inZone() &&
                        turret.atTargetAngle()) || operator.wasJustPressed(GamepadKeys.Button.A)
                ) {
                    state = OutState.SHOOT;
                    lastShots = shooter.getCurrentShots();
                    shotSlot = spindexer.currentSlot();
                    spindexer.spinShoot();

                    // try to rapid-shoot if we're close enough
                    if (true) { //geometricDistance < shooter.FAR_DISTANCE) {
                        for (int x=0; x < spindexer.artifactCount(); x++) {
                            spindexer.spinShoot();
                        }
			// 'bonus' shot, required on old robot to get
			// fully past at high power, but do we need it
			// for v3?
			///spindexer.spinShoot();
                    }
                }
            } else if (state == OutState.SHOOT) {
		if (spindexer.isStuck()) {
		    // go 'back' to the nearest 120-degree increment
		    double angle = spindexer.currentAngle % 120;
		    spindexer.targetAngle = (int)(spindexer.targetAngle - angle);
		    state = OutState.DONE;
		}
		
                // try just not caring about "did a shot really go up"
                // for this -- so we're just trusting the spindexer's
                // notion of how many balls
                if (spindexer.atTarget()) { //shooter.getCurrentShots() > lastShots) {
                    if (spindexer.isEmpty()) {
			startPause = time;
                        state = OutState.PAUSE;
                    } else {
                        state = OutState.WAIT_SHOOT;
                    }
                }
            } else if (state == OutState.PAUSE) {
		double elapsed = time - startPause;
		if (elapsed > SHOOT_PAUSE_WAIT) {
		    state = OutState.DONE;
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
            intake.stop();
            if (!interrupted) {
                if (isAuto()){
                    turret.noLock();
                    spindexer.spinModeIndex();
                }
                else {
                    turret.noLock();
                    shooter.manualShootRpm();
                    spindexer.spinModeIndex();
                }
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


    public class NoIntake extends CommandBase {
        public NoIntake() {
            addRequirements(intake);
        }
        public void initialize() {
            intake.stop();
        }
        public boolean isFinished() {
            return true;
        }
    }


    public class SpindexMode extends CommandBase {
        public SpindexMode() {
            addRequirements(spindexer);
        }
        public void initialize() {
            spindexer.spinModeIndex();
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
            // Add this: if (robotHeading < 0) robotHeading = robotHeading + 360;

            double turretX = robotX - (Math.cos(Math.toRadians(robotHeading)) * ROBOT_CENTER_TO_TURRET_INCHES);
            double turretY = robotY - (Math.sin(Math.toRadians(robotHeading)) * ROBOT_CENTER_TO_TURRET_INCHES);

            double obeliskHeading = Math.toDegrees(
                Math.atan2(141.0 - turretY, 70.5 - turretX)
                );

            // for close-zone autos, we actually look at the _side_ of
            // the obelisk and adjust .. for "blue-side" we move 5
            // degrees right and then can in theory only see the side
            // one .. for "red-size" we turn a bunch left (45?) and
            // then see the "other" side of the obelisk


            turret.faceObelisk(obeliskHeading);
        }
        public boolean isFinished(){
            // we look for up to 1 second, or until we see an Obelisk pattern
            return (time - startTime > 1.0) || (turret.pattern != -1);
        }
        public void end(boolean interrupted) {
            turret.noLock();
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
             /*   if (spindexer.isFull() || spindexer.isEmpty()){
                    //do nothing
                }
                if (spindexer.countArtifacts() == 1){
                    if (spindexer.firstFullSlot() == spindexer.currentSlot()){
                        spindexer.spinIndex();
                        spindexer.spinIndex();
                    }
                    int previous = spindexer.currentSlot() - 1;
                    if (previous < 0 ){
                        previous = 2;
                    }
                    if (spindexer.firstFullSlot() == previous){
                        spindexer.spinIndex();
                    }
                    else{
                        //do nothing
                    }
                }
                if (spindexer.countArtifacts() == 2){
                    if (spindexer.firstEmptySlot() == spindexer.currentSlot()){
                        spindexer.spinIndex();
                    }
                    int prev = spindexer.currentSlot() +1;
                    if (prev < 0){
                        prev = 2;
                    }
                    if (spindexer.firstEmptySlot() == prev){
                        spindexer.spinIndex();
                        spindexer.spinIndex();
                    }
                    else{
                        //do nothing
                    }
                } */
        }
        public void execute(){

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

        recentPositions.addLast(drive.getPosition());
        while (recentPositions.size() > 5) {
            recentPositions.removeFirst();
        }
    }

    public boolean inZone(){
        if (isAuto()) {
            double x  = drive.getPosition().getX(DistanceUnit.INCH);
            double y  = drive.getPosition().getY(DistanceUnit.INCH);
            if (getAlliance() == Alliance.BLUE) {
                // Blue: are we in the upper left quadrant of the field, and are we within the triangle, or we are in the far zone "box"
                if ((y >= 63.5 && x <= 70.5 && x + y >= 131.1) || (y < 17 && x > 24 && x < 36)) {
                    return true;
                }
            }
            else if (getAlliance() == Alliance.RED) {
                // Red: are we in the upper right quadrant of the field, and in the triangle, or we are in the far zone "box"
                if ((y >= 63.5 && x >= 70.5 && y >= x - 9.9) || (y < 17 && x > 36 && x < 48)) {
                    return true;
                }
            }
            return false;
        }
        else {
            return true;
        }
    }

    // based on recentPositions, predict our Post2D in "t" seconds
    // from now (just x, y works velocity)
    protected Pose2D predictPose(double t) {
        return drive.getPosition();
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
    private double timeOfFlight(double distance){
        double t = 0.00008 * distance * distance - 0.0093 * distance + 0.8551;
        return t;
    }

    protected void addTelemetry(HyperTelemetry telem) {
    }

    protected void logTelemetry() {
        //TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry); //, pack);
        telem.log("elapsed", runtime.seconds());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());
        telem.log("geometric-target", geometricTargetHeading);
        telem.log("geometric-distance", geometricDistance);
        telem.log("alliance", getAlliance());
        telem.log("zone", getStartZone());
        telem.log("shooter-ready-to-shoot", shooter.readyToShoot());
        telem.log("time-of-flight", timeOfFlight(geometricDistance));
        telem.log("aim-offset-x", aimOffsetX);
        telem.log("aim-offset-y", aimOffsetY);
        telem.log("predicted-x", predictedX);
        telem.log("predicted-y", predictedY);
        telem.log("in-zone", inZone());
        telem.log("loops", loops);

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
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? 94 - 7.4 : 47.5 + 7.4, 8.0984, AngleUnit.DEGREES, 90));
        loops = 0;
    }

    @Override
    public void init_loop() {
        clearCache();

        // runs while the robot is "on" but we haven't pressed "play" yet
        turret.read_sensors(0.0);
        telemetry.addData("turret-ticks", turret.revEncoder.getCurrentPosition());
        telemetry.addData("turret-servo", turret.getServoAngle());
        telemetry.addData("turret-angle", turret.getTurretAngle());
        telemetry.addData("spindexer-ticks", spindexer.spindexerMotor.getCurrentPosition());
      /*  telemetry.addData("Turret Servo Right", turret.getServoAngle());
        telemetry.addData("Turret Servo Left", turret.getOtherServoAngle());
        telemetry.update(); */
    }

    @Override
    public void loop() {
        clearCache();
        readControls();
        readSensors();
        if (!isAuto()){
            if (operatorPattern == 0){
                turret.motifLight.setPosition(0);
            }
            if (operatorPattern == 1){
                turret.motifLight.setPosition(0.71);
            }
            if (operatorPattern == 2){
                turret.motifLight.setPosition(0.61);
            }
        }

        if (!minuteWarning && runtime.seconds() > 60){
            minuteWarning = true;
            operator.gamepad.rumble(300);
        }
        if (!endgameWarning && runtime.seconds() > 90){
            endgameWarning = true;
            operator.gamepad.rumble(600);
        }
        // We need to rotate the FTC coordinate system 90 degrees to
        // get the pedro pathing system, and Offset by 72 inches (70.5)

        double robotHeading = drive.getPosition().getHeading(AngleUnit.DEGREES);
        double robotX = drive.getPosition().getX(DistanceUnit.INCH);
        double robotY = drive.getPosition().getY(DistanceUnit.INCH);

        double tof = SHOOT_PREDICT; // timeOfFlight(geometricDistance);
        aimOffsetX = drive.x_velocity * tof;
        aimOffsetY = drive.y_velocity * tof;
        robotX += aimOffsetX;
        robotY += aimOffsetY;
        predictedX = robotX;
        predictedY = robotY;

        // We prefer angels from 0-360, but atan2 likes 180 to -180
      //  if (robotHeading < 0) robotHeading = robotHeading + 360;

        // we need to offset the robot x and y values to be at the
        // center of the turret.
        double turretX = robotX - (Math.cos(Math.toRadians(robotHeading)) * ROBOT_CENTER_TO_TURRET_INCHES);
        double turretY = robotY - (Math.sin(Math.toRadians(robotHeading)) * ROBOT_CENTER_TO_TURRET_INCHES);

        // AAAAAAaaaaaa! okay, so FTC co-ordinate system says the
        // field is 144x144 inches. This is not true, it is actually
        // 141.5 inches. The field-center is (70.5, 70.5) NOT (72, 72)
        // if we measure from tile-edges.

        //double targetX = turret.target.fieldPosition.get(1);
        //double targetY = -turret.target.fieldPosition.get(0);
        // TODO: red vs blue targets
        double targetX = GEOM_TARGET_X;
        double targetY =  GEOM_TARGET_Y;


        if (getAlliance() == Alliance.RED){
            targetX = 141 - GEOM_TARGET_X;
        }

        double distanceA = targetX - turretX;
        double distanceB = targetY - turretY;
        geometricDistance = Math.sqrt((distanceA * distanceA) + (distanceB * distanceB));

        if (shooter.autoRpm && geometricDistance > shooter.FAR_DISTANCE){
            if (getAlliance() == Alliance.BLUE) {
                targetX = FAR_TARGET_X_BLUE;
            }
            else {
                targetX = 141 - FAR_TARGET_X_RED;
            }
        }
        geometricTargetHeading = Math.toDegrees(
            Math.atan2(targetY - turretY,
                       targetX - turretX)
        );

        turret.robotHeading = robotHeading;
        if (false && turret.isLocked(time)) {
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
