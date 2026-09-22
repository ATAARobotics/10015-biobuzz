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
    double strafe;
    double forward;
    double turn;

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
    public static double GEOM_TARGET_X = 0;
    public static double GEOM_TARGET_Y = 141;
    public int operatorPattern = 0;

    public static double FAR_TARGET_X_BLUE = 9.5;
    public static double FAR_TARGET_X_RED =  3.5;
    public static double SHOOT_PREDICT = 0.250;//0.450;

    public static double SHOOT_PAUSE_WAIT = 0.250;

    public enum InState {FIRST_TWO, SPIN, THIRD, WAIT_BEFORE_PIN, WAIT_AFTER_PIN, WAIT_SPIT, DONE};
    public enum OutState {WAIT_SHOOT, SHOOT, PAUSE, DONE};

    private InState inState = InState.DONE;
    private double startPinWait = 0.0;
    private double startSpitWait = 0.0;

    public OutState lastOutState = OutState.DONE;

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

        battery = hardwareMap.voltageSensor.get("Control Hub");

        recentPositions = new LinkedList<Pose2D>();

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
      //  CommandScheduler.getInstance().registerSubsystem(drive);

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

    protected void readSensors() {
        drive.read_sensors(time);

        recentPositions.addLast(drive.getPosition());
        while (recentPositions.size() > 5) {
            recentPositions.removeFirst();
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
	telem.log("in-state", inState);
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
        telem.log("time-of-flight", timeOfFlight(geometricDistance));
        telem.log("aim-offset-x", aimOffsetX);
        telem.log("aim-offset-y", aimOffsetY);
        telem.log("predicted-x", predictedX);
        telem.log("predicted-y", predictedY);
        telem.log("loops", loops);
        telem.log("in-state", inState);

        double fps = loops / runtime.seconds();
        telem.logDrivers("average fps", fps);

        drive.addTelemetry(telem);
        addTelemetry(telem);

        telem.update();
    }

    @Override
    public void start() {
        runtime.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? 94 - 7.179 : 47.25 + 7.179, 7.19, AngleUnit.DEGREES, 90));
        loops = 0;
    }

    @Override
    public void init_loop() {
        clearCache();

        // runs while the robot is "on" but we haven't pressed "play" yet
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
        if (!endgameWarning && runtime.seconds() > 90){
            endgameWarning = true;
            operator.gamepad.rumble(600);
        }
        // We need to rotate the FTC coordinate system 90 degrees to
        // get the pedro pathing system, and Offset by 70.5 inches

        double robotHeading = drive.getPosition().getHeading(AngleUnit.DEGREES);
        double robotX = drive.getPosition().getX(DistanceUnit.INCH);
        double robotY = drive.getPosition().getY(DistanceUnit.INCH);

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        //CommandScheduler.getInstance().run();
        strafe = driver.getRightX();
        forward = -driver.getRightY();
        turn = 0;
        drive.drivebase.driveRobotCentric(strafe, forward, turn);


        logTelemetry();
    }

    @Override
    public void stop() {
        drive.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
