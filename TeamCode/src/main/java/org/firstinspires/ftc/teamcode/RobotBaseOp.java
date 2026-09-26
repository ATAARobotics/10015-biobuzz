package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.opencv.core.Scalar;

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

    ColorBlobLocatorProcessor colorLocator;
    VisionPortal portal;

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
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(
                        new ColorRange(
                                ColorSpace.HSV,
                                new Scalar(15, 150, 100),
                                new Scalar(35, 255, 255)))   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(3)       // Expand blobs to fill any divots on the edges
                .setErodeSize(7)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();
        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution
         *      that is supported by your camera.  This will improve overall performance and reduce
         *      latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        portal = new VisionPortal.Builder()
	    .addProcessor(colorLocator)
	    .setCameraResolution(new Size(640, 480))
	    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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

	// testing driver controls
        strafe = driver.getRightX();
        forward = -driver.getRightY();
        turn = 0;

	// vision stuff: look for yellow / pollen blobs
	List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
	
	ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.7, 1, blobs);  // filter out non-circles.

	telemetry.addData("Pollen Count", blobs.size());
    double bestSize = 0;
    ColorBlobLocatorProcessor.Blob bestBlob = null;
	for (ColorBlobLocatorProcessor.Blob b: blobs) {
	    Circle circleFit = b.getCircle();
        if (bestBlob == null || b.getContourArea() > bestSize){
            bestSize = b.getContourArea();
            bestBlob = b;
        }
	}
    if (bestBlob != null){
        double x = bestBlob.getCircle().getX();
        x -= 320;
        x /= 320;
        turn = x;
    }


	// actually do the drive command for this loop
        drive.drivebase.driveRobotCentric(strafe, forward, turn);

	loopExtra();
	
        logTelemetry();
    }

    @Override
    public void stop() {
        drive.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
