package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Turret Testing", group="Opmode")
public class TurretTesting extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    VoltageSensor battery;
    Shooter shooter;
    Turret turret;
    ElapsedTime  runtime = new ElapsedTime();

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;

    public enum Alliance {RED, BLUE}
    public Alliance getAlliance(){return Alliance.BLUE;}
    boolean isRedAlliance;
    AprilTagMetadata target;
    double distToAprilTag;

    @Override
    public void init() {
        isRedAlliance = getAlliance() == Alliance.RED;
      //  target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(21);

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
                .setCamera(hardwareMap.get(WebcamName.class,"elp"))
                .addProcessor(april_tags)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        //telemetry.addData("Pinpoint Firmware Version", drive.pinpoint.getDeviceVersion());
        //telemetry.update();

        battery = hardwareMap.voltageSensor.get("Control Hub");

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        // FIXME TODO we had a "CommandScheduler.getInstance().reset()"
        // here at some point, but: do we need that? Also deleting
        // laser-sensor seemed to fix our previous problem anyway

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(turret);

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(shooter, shooter.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(turret, turret.new HumanInputs(operator, driver));
    }

    @Override
    public void start() {
        runtime.reset();
        turret.reset();
        // this is the far-zone starting position, against the wall with robot facing "north" / away from audience
    }

    @Override
    public void init_loop() {
        // runs while the robot is "on" but we haven't pressed "play" yet
        telemetry.addData("Turret Servo Angle", turret.getServoAngle());
        telemetry.update();
    }

    @Override
    public void loop() {
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        shooter.read_sensors(time);
        //turret.read_sensors(time);
        //intake.read_sensors(time);

//      target.fieldOrientation.toOrientation(AxesReference.EXTRINSIC,AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle-90
        TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry, pack);

        List<AprilTagDetection> detections = april_tags.getDetections();
        telem.logBoth("april-tags", detections.size());
        for (AprilTagDetection tag : detections) {
            if (tag.id == target.id){
                //  drive.april_bearing = drive.getPosition().getHeading(AngleUnit.DEGREES) - tag.ftcPose.bearing;
                telem.logBoth("april-tag-target", tag.ftcPose.range);
                //range(distance)is in inches, maybe convert to centi
                telem.logBoth("april-tag-bearing", tag.ftcPose.bearing);
                distToAprilTag = tag.ftcPose.range;
                turret.faceRobotAngle(tag.ftcPose.bearing + turret.currentTurretAngle);
                telem.logBoth("april-tag-distance", distToAprilTag);
            }
        }

        //telemetry.addData("Camera", drive.cameraOn);

        // Run the CommandScheduler instance (note: this will call
        // ".periodic()" on all registered subsystems, which is the
        // correct place to do "per-loop" things)
        CommandScheduler.getInstance().run();


        telem.log("elapsed", runtime.toString());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());

        shooter.addTelemetry(telem);
        turret.addTelemetry(telem);

        // log some drivetrain information always too

        telemetry.update();
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.stop();

        // Cancel all previous commands
        CommandScheduler.getInstance().reset();
    }
}

