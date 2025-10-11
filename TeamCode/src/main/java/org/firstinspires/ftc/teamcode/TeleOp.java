package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Config
public abstract class TeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    VoltageSensor battery;
    Drive drive;
    ElapsedTime  runtime = new ElapsedTime();

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;

    public enum Alliance {RED, BLUE};
    public abstract Alliance getAlliance();

    AprilTagMetadata target;

    @Override
    public void init() {
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(getAlliance() == Alliance.BLUE ? 20 : 24);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        //AprilTagLibrary decode_tags = ;
        // game manual says april tag family is 36h11
        april_tags = new AprilTagProcessor.Builder()
                //.setTagLibrary(decode_tags)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"DubbleBubble Webcam"))
                .addProcessor(april_tags)
                .setCameraResolution(new Size(640, 480))
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        drive = new Drive(hardwareMap, driver);
        telemetry.addData("Pinpoint Firmware Version", drive.pinpoint.getDeviceVersion());
        telemetry.update();

        battery = hardwareMap.voltageSensor.get("Control Hub");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
    }

    @Override
    public void start() {
        runtime.reset();
        //drive.reset();
        // set starting position
        drive.setPosition(new Pose2D(DistanceUnit.METER, 0.435, -1.61, AngleUnit.DEGREES, 0));
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play" yet
    }

    @Override
    public void loop() {
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        drive.read_sensors(time);

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // Send telemetry messages to explain controls and show robot status
        // to see telemetry in Webots, right click on your robot and select "Show Robot Window"
        Pose2D drivePosition = drive.getPosition();

        drive.april_bearing = Math.toDegrees(Math.atan2(
                drive.getPosition().getX(DistanceUnit.METER) - target.distanceUnit.toMeters(target.fieldPosition.get(1)),
                -drive.getPosition().getY(DistanceUnit.METER) - target.distanceUnit.toMeters(target.fieldPosition.get(0))));
//      target.fieldOrientation.toOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle-90

        List<AprilTagDetection> detections = april_tags.getDetections();
        for (AprilTagDetection tag : detections) {
            if (tag.id == target.id){
                drive.april_bearing = tag.ftcPose.bearing + drive.getPosition().getHeading(AngleUnit.DEGREES);
                telemetry.addData("target", tag.ftcPose.range);
                //range(distance)is in inches, maybe convert to centi
                telemetry.addData("bearing", tag.ftcPose.bearing);
            }
        }


        // FIXME TODO put into FTC Dashboard too, for most of this
        telemetry.addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.getX(DistanceUnit.METER), drivePosition.getY(DistanceUnit.METER), drivePosition.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.getX(DistanceUnit.METER), drivePosition.getY(DistanceUnit.METER), drivePosition.getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }

    @Override public void stop() {
        drive.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}
