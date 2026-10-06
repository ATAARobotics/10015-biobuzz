package org.firstinspires.ftc.teamcode;

// import android.graphics.Color;
import android.util.Size;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.opencv.imgproc.Imgproc;

//import org.opencv.core.Scalar;

import java.util.LinkedList;
import java.util.List;


@Configurable
public abstract class RobotBaseOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;

    VoltageSensor battery;
    List<LynxModule> allHubs;
    int loops;

    Drive drive;
    double strafe;
    double forward;
    double turn;
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

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

	//CommandScheduler.getInstance().registerSubsystem(drive);

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

// camera and vision setup

        pollenDetector = new PollenDetector();

        portal = new VisionPortal.Builder()
                .addProcessor(pollenDetector)
                .setCameraResolution(new Size(640, 480))
                .setCamera(
                        hardwareMap.get(
                                WebcamName.class,
                                "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
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
        loops++;
    }

    protected void logTelemetry() {
        HyperTelemetry telem = new HyperTelemetry(telemetry);
        //telem.log("elapsed", runtime.seconds());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());
        telem.log("alliance", getAlliance());

        //double fps = loops / runtime.seconds();
        //telem.logDrivers("average fps", fps);

        drive.addTelemetry(telem);

        telem.update();
    }

    @Override
    public void start() {
        //runtime.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
        drive.setPosition(new Pose2D(DistanceUnit.INCH, isRedAlliance() ? 94 - 7.179 : 47.25 + 7.179, 7.19, AngleUnit.DEGREES, 90));
        loops = 0;
    }

    @Override
    public void init_loop() {
        clearCache();
    }

    // this is called once per loop for subclasses that want to keep
    // the rest of this class' "loop()" but also want to do their own
    // thing
    protected void loopExtra() {
    }

    @Override
    public void loop() {
        clearCache();
        readControls();
        readSensors();

        pollenTurnPID.setPID(POLLEN_P,POLLEN_I,POLLEN_D);

	// testing driver controls
        strafe = driver.getRightX();
        forward = -driver.getRightY();
        turn = 0;

// vision stuff: look for yellow / pollen blobs

        List<PollenDetector.PollenBlob> blobs =
                pollenDetector.getBlobs();

        telemetry.addData(
                "Pollen Count",
                blobs.size());


        /*
         * PollenDetector sorts the list from
         * largest area to smallest area.
         */

        PollenDetector.PollenBlob bestBlob = null;

        if (!blobs.isEmpty()) {

            bestBlob = blobs.get(0);
        }


        /*
         * Turn toward the largest pollen.
 Imgproc.erode(mask, mask, erodeKernel);Imgproc.erode(mask, mask, erodeKernel);        */

        if (bestBlob != null) {

            double halfWidth =
                    pollenDetector.getFrameWidth()
                            / 2.0;
            frameCenter = pollenDetector.getFrameWidth() / 2.0;
        turn =
                   /* (bestBlob.x - halfWidth)
                            / halfWidth;*/
                    -pollenTurnPID.calculate(bestBlob.x, frameCenter);


            telemetry.addData(
                    "Pollen",
                    "x=%.0f y=%.0f area=%.0f circ=%.2f pollenDist=%.2f",
                    bestBlob.x,
                    bestBlob.y,
                    bestBlob.area,
                    bestBlob.circularity,
                    bestBlob.pollenDistance);
        }

	// actually do the drive command for this loop
        drive.drivebase.driveRobotCentric(strafe, forward, turn);

	loopExtra();
	
        logTelemetry();
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
