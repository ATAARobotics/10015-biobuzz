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
    public enum TippedSide {AUDIENCE, SCORING, UNKNOWN}
    public TippedSide tippedSide = TippedSide.UNKNOWN;
    public static double POLLEN_P = 0.0013;
    public static double POLLEN_I = 0;
    public static double POLLEN_D = 0.0001;
    private final PIDController pollenTurnPID = new PIDController(POLLEN_P, POLLEN_I, POLLEN_D);
    Servo indicatorLight;
    double RED = 0.28;
    double GREEN = 0.5;
    AprilTagProcessor tags;

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
        indicatorLight = hardwareMap.get(Servo.class, "indicator");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

	CommandScheduler.getInstance().registerSubsystem(drive);

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

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tags)
                .build();
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
        clearCache();
        readControls();
        readSensors();
        tippedSide = TippedSide.UNKNOWN;
        telemetry.addData("Detections", tags.getDetections().size());
        for (AprilTagDetection tag : tags.getDetections()) {

            if (tag.ftcPose == null) continue;

           if (tag instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) tag;
                telemetry.addData("Cluster", cluster.metadata.name);
                if(cluster.metadata.name.startsWith("B") && getAlliance() == Alliance.BLUE){
                    if (drive.getPosition().getY(DistanceUnit.INCH) < 48 && cluster.metadata.shortName.contains("AUDIENCE")){
                        tippedSide = TippedSide.AUDIENCE;
                    }else if (drive.getPosition().getY(DistanceUnit.INCH) > 96  && cluster.metadata.shortName.contains("SCORING")){
                        tippedSide = TippedSide.SCORING;
                    }
                } else if (cluster.metadata.name.startsWith("R") && getAlliance() == Alliance.RED){
                    if (drive.getPosition().getY(DistanceUnit.INCH) < 48  && cluster.metadata.shortName.contains("AUDIENCE")){
                        tippedSide = TippedSide.AUDIENCE;
                    }else if (drive.getPosition().getY(DistanceUnit.INCH) > 96  && cluster.metadata.shortName.contains("SCORING")){
                        tippedSide = TippedSide.SCORING;
                    }
                }
                telemetry.addData("Tipped Side:", tippedSide);
            }

           /* telemetry.addData("Distance", "%.1f inches", tag.ftcPose.range);
            telemetry.addData("Bearing", "%.1f degrees", tag.ftcPose.bearing);
            telemetry.addData("Roll", "%.1f degrees", tag.ftcPose.roll);
            telemetry.addData("Pitch", "%.1f degrees", tag.ftcPose.pitch);
            telemetry.addData("Yaw", "%.1f degrees", tag.ftcPose.yaw);
            telemetry.addLine();*/
        }

	if (!isAuto()) {
	    pollenTurnPID.setPID(POLLEN_P, POLLEN_I, POLLEN_D);

	    // testing driver controls
	    strafe = driver.getRightX();
	    forward = -driver.getRightY();
	    turn = 0;

	    // vision stuff: look for yellow / pollen blobs

	 //   List<PollenDetector.PollenBlob> blobs =
	//	pollenDetector.getBlobs();

	   /* telemetry.addData(
			      "Pollen Count",
			      blobs.size());


	    /*
	     * PollenDetector sorts the list from
	     * largest area to smallest area.
	     */
/*
	    PollenDetector.PollenBlob bestBlob = null;

	    if (!blobs.isEmpty()) {

		bestBlob = blobs.get(0);
	    }


	    /*
	     * Turn toward the largest pollen.
	     Imgproc.erode(mask, mask, erodeKernel);Imgproc.erode(mask, mask, erodeKernel);        */

	   // if (bestBlob != null) {

		//double halfWidth =
		//    pollenDetector.getFrameWidth()
		   // 2.0;
		//frameCenter = pollenDetector.getFrameWidth() / 2.0;
		//turn =
		    /* (bestBlob.x - halfWidth)
		       / halfWidth;*/
                /*
		    -pollenTurnPID.calculate(bestBlob.x, frameCenter);


		telemetry.addData(
				  "Pollen",
				  "x=%.0f y=%.0f area=%.0f circ=%.2f pollenDist=%.2f",
				  bestBlob.x,
				  bestBlob.y,
				  bestBlob.area,
				  bestBlob.circularity,
				  bestBlob.pollenDistance);
	    }*/

	    // let the command scheduler do stuff

	    // actually do the drive command for this loop
	    // todo: incorporate into commands / default commands
	    drive.drivebase.driveRobotCentric(strafe, forward, turn);

	    logTelemetry();
	}
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
