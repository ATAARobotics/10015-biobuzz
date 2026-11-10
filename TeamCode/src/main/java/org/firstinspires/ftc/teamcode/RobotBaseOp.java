package org.firstinspires.ftc.teamcode;

// import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TipDetector;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.Circle;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ColorSpace;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.vision.PollenDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.imgproc.Imgproc;

//import org.opencv.core.Scalar;

import java.util.LinkedList;
import java.util.List;


@Configurable
public abstract class RobotBaseOp extends OpMode {
    HyperTelemetry telem;
    GamepadEx driver;
    GamepadEx operator;

    VoltageSensor battery;
    List<LynxModule> allHubs;

    int loops;
    Timing.Stopwatch runtime;

    Drive drive;
    Shooter shooter;
    double frameCenter;

    PollenDetector pollenDetector;
    VisionPortal portal;

    public enum StartZone {NEAR, FAR}
    public enum Alliance {RED, BLUE}
    public abstract Alliance getAlliance();
    public static double POLLEN_P = 0.0013;
    public static double POLLEN_I = 0;
    public static double POLLEN_D = 0.0001;
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

        drive = new Drive(hardwareMap, isRedAlliance(), isAuto());
       // tipDetector = new TipDetector( tags, telemetry, alliance, drive);//FIXME give tip decetor hyper telem instead of telemetry
        //shooter = new Shooter(hardwareMap, telem);

        battery = hardwareMap.voltageSensor.get("Control Hub");
	//        indicatorLight = hardwareMap.get(Servo.class, "indicator");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

	CommandScheduler.getInstance().registerSubsystem(drive);
   // CommandScheduler.getInstance().registerSubsystem(tipDetector);
   // CommandScheduler.getInstance().registerSubsystem(shooter);

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

	double seconds = runtime.deltaTime() / 1000.0;
        double fps = loops / seconds;
	telem.logBoth("fps", fps);

        drive.addTelemetry(telem);

        telem.update();
    }

    @Override
    public void start() {
        //runtime.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? 94 - 7.179 : 47.25 + 7.179, 7.19, AngleUnit.DEGREES, 90));
        loops = 0;
	runtime.start();
	//  shooter.start();
    }

    @Override
    public void init_loop() {
        clearCache();
    }

    class Shoot extends CommandBase{
        @Override
        public void execute(){
            indicatorLight.setPosition(GREEN);
        }
        @Override
        public boolean isFinished(){
            return true;
        }
    }
    public Command turnLightGreen() {
        return new Shoot();
    }
    class Intake extends CommandBase{
        @Override
        public void execute(){
            indicatorLight.setPosition(RED);
        }
        @Override
        public boolean isFinished(){
            return true;
        }
    }
    public Command turnLightRed() {
        return new Intake();
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
