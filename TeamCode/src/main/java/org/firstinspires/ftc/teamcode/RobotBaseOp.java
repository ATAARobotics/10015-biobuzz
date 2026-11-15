package org.firstinspires.ftc.teamcode;

// import android.graphics.Color;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.util.Timing;
import java.util.concurrent.TimeUnit;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.Circle;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ColorSpace;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.vision.PollenDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//import org.opencv.core.Scalar;

import java.util.List;


@Configurable
public abstract class RobotBaseOp extends OpMode {
    double xOffset = 8.124;
    double yOffset = 8.649;
    HyperTelemetry telem;
    GamepadEx driver;
    GamepadEx operator;

    VoltageSensor battery;
    List<LynxModule> allHubs;

    int loops;
    Timing.Stopwatch runtime;

    Drive drive;
    Transfer transfer;
    Shooter shooter;
    Turret turret;
    double frameCenter;

    PollenDetector pollenDetector;
    VisionPortal portal;

    public enum StartZone {NEAR, FAR}
    public enum Alliance {RED, BLUE}
    public abstract Alliance getAlliance();
    public enum ShootState{WAIT_SHOOT, SHOOT, DONE}
    public static double POLLEN_P = 0.0013;
    public static double POLLEN_I = 0;
    public static double POLLEN_D = 0.0001;
    public static double RPM_TOLERANCE = 100;
    private final PIDController pollenTurnPID = new PIDController(POLLEN_P, POLLEN_I, POLLEN_D);
    Servo indicatorLight;
    double RED = 0.28;
    double GREEN = 0.5;
    AprilTagProcessor tags;
    TipDetector tipDetector;
    Alliance alliance;
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
        telem =  new HyperTelemetry(telemetry);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drive = new Drive(hardwareMap, isRedAlliance(), isAuto(), telem);
       // tipDetector = new TipDetector( tags, telemetry, alliance, drive);//FIXME give tip decetor hyper telem instead of telemetry
        shooter = new Shooter(hardwareMap, telem);
        transfer = new Transfer(hardwareMap, telem);
        turret = new Turret(hardwareMap, telem);


        battery = hardwareMap.voltageSensor.get("Control Hub");
	//        indicatorLight = hardwareMap.get(Servo.class, "indicator");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

	CommandScheduler.getInstance().registerSubsystem(drive);
    CommandScheduler.getInstance().registerSubsystem(transfer);
   // CommandScheduler.getInstance().registerSubsystem(tipDetector);
    CommandScheduler.getInstance().registerSubsystem(shooter);
    CommandScheduler.getInstance().registerSubsystem(turret);

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
        loops = 0;
	runtime = new Timing.Stopwatch(TimeUnit.MILLISECONDS);

	// camera and vision setup


	/*pollenDetector = new PollenDetector();


            portal = new VisionPortal.Builder()
                    .addProcessor(pollenDetector)
                    .setCameraResolution(new Size(640, 480))
                    .setCamera(
                            hardwareMap.get(
                                    WebcamName.class,
                                    "Webcam 1"))
                    .build();*/

            telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        tags = new AprilTagProcessor.Builder().build();
	/*
        portal = new VisionPortal.Builder()
	    .setCamera(hardwareMap.get(WebcamName.class, "elp"))
	    .addProcessor(tags)
	    .build();
      //  tipDetector = new TipDetector(tags, telemetry, getAlliance(),drive);
      */
        }


    protected void readSensors() {
        drive.readSensors(time);
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
    }

    protected void logTelemetry() {
        //telem.log("elapsed", runtime.seconds());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());
        telem.log("alliance", getAlliance());
        telem.logBoth("X:", drive.currentPosition.getX(DistanceUnit.INCH));
        telem.logBoth("Y:", drive.currentPosition.getY(DistanceUnit.INCH));
        telem.logBoth("Heading:", drive.currentPosition.getHeading(AngleUnit.DEGREES));
        telem.logBoth("TargetAngle", drive.targetAngle);
        telem.logBoth("currentAngle", drive.currentPosition.getHeading(AngleUnit.DEGREES));

	double seconds = runtime.deltaTime() / 1000.0;
        double fps = loops / seconds;
	telem.logBoth("fps", fps);
        telem.update();
    }

    @Override
    public void start() {
        //runtime.reset();
        // This is right above the loading zone for blue, touching the wall. Red is below the loading zone. Both face the hive.
        //drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? drive.blueXToRedX(141 - yOffset): 141 - yOffset,
              //  isRedAlliance() ? drive.blueYToRedY(47 + xOffset):47 + xOffset,
              // AngleUnit.DEGREES, isRedAlliance() ? drive.blueHeadingToRedHeading(180) :180));

        if(OpmodeStorage.autoRan==true){
            drive.follower.setPose(OpmodeStorage.autonomousEndPose);
        }
        else{
            if (!isAuto()) {
            drive.follower.setPose(new Pose(isRedAlliance() ? drive.blueXToRedX(141.5 - yOffset) : 141.5 - yOffset,
                    isRedAlliance() ? drive.blueYToRedY(47.5 + xOffset) : 47.5 + xOffset,
                    isRedAlliance() ? Math.toRadians(drive.blueHeadingToRedHeading(180)) : Math.toRadians(180)));
            }
        }
        drive.follower.update();
        loops = 0;
	runtime.start();
	//  shooter.start();
    }

    @Override
    public void init_loop() {
        clearCache();
    }
    public class StartAutoIntake extends CommandBase{
        public StartAutoIntake(){
            addRequirements(transfer);
        }
        @Override
        public void execute() {
            transfer.pollenBlocked();
            transfer.runIntake();
            transfer.runTransfer();
        }
        @Override
        public boolean isFinished(){
            return true;
        }
    }
    public class StopAutoIntake extends CommandBase{
        public StopAutoIntake(){
            addRequirements(transfer);
        }
        @Override
        public void execute(){
            transfer.stopIntake();
            transfer.stopTransfer();
            transfer.setCrashBarNothing();
        }
        @Override
        public boolean isFinished(){
            return true;
        }
    }
    public class AutoShoot extends CommandBase{
        private ShootState state;
        public AutoShoot() {
            addRequirements(shooter);
            addRequirements(transfer);
            addRequirements(turret);
        }
        @Override
        public void initialize(){
            state = ShootState.WAIT_SHOOT;
        }
        @Override
        public void execute(){
            if (state == ShootState.WAIT_SHOOT){
                shooter.setRpm();
                //fixme: Add turret aim method here
                if (readyToShoot()) {
                    state = ShootState.SHOOT;
                    transfer.runTransfer();
                    transfer.runIntake();
                    transfer.pollenUnblocked();
                }
            } else if (state == ShootState.SHOOT){
                state = ShootState.DONE;
            }
        }
        public boolean isFinished(){
            return state == ShootState.DONE;
        }
        public void end(boolean interrupted){
            if (!interrupted){
                shooter.stopRpm();
                //fixme: turret stop tracking
                transfer.stopTransfer();
                transfer.stopIntake();
                transfer.pollenBlocked();
            }
        }
    }
    public boolean readyToShoot(){
        //fixme: make it so turret has to be at position too
        if (shooter.targetRpm - shooter.recentVelocity.getLast().velocity <= RPM_TOLERANCE ||
            shooter.targetRpm - shooter.recentVelocity.getLast().velocity >= RPM_TOLERANCE){
            return true;
        } else{
            return false;
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

    @Override
    public void loop() {
        loops++;
        clearCache();
        readControls();
        readSensors();

	// subsystems can log things
	logTelemetry();

	// let the command scheduler do stuff
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        drive.stop();

        if (portal != null) {
            portal.close();
            portal = null;
        }
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
