package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public abstract class Auto extends OpMode {
    VoltageSensor battery;
    Drive drive;
    Shooter shooter;
    GamepadEx driver;
    GamepadEx operator;
    double pauseTime = 0;
    ElapsedTime runtime = new ElapsedTime();

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;

    public enum Alliance {RED, BLUE};
    public abstract Alliance getAlliance();
    boolean isRedAlliance;
    AprilTagMetadata target;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driver.readButtons();
        operator.readButtons();
        isRedAlliance = getAlliance() == Alliance.RED;

        // (Do not remove this, we absolutely have problems without cancelling this)
        // Cancel all previous commands
        CommandScheduler.getInstance().reset();

        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
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

        drive = new Drive(hardwareMap, isRedAlliance);
        shooter = new Shooter(hardwareMap);

        battery = hardwareMap.voltageSensor.get("Control Hub");

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(shooter);

        drive.reset();
        shooter.reset();
    }

    @Override
    public void init_loop() {
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){

        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){

        }

    }

    // origin is center of the field, in meters
    // Looking from audience side, axis X is pointing left and axis Y is pointint forward
    @Override
    public void start() {
        // OPTION 1: starting position is touching audience field perimeter wall
//        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 0.435 : -0.435, -1.61, AngleUnit.DEGREES, 0));

        // OPTION 2: starting position is over the center of a launch line touching own alliance's goal:
        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 1.3 : -1.3, 1.3, AngleUnit.DEGREES, isRedAlliance ? 135 : -135));

        SequentialCommandGroup auto_commands = new SequentialCommandGroup(
                drive.moveQuickly(isRedAlliance ? 1.0 : -1.0,1.0, isRedAlliance ? 135 : -135),
                shooter.shoot(3),
                drive.moveQuickly(isRedAlliance ? 1.2 : -1.2,0.6, 180)
        );

        // schedule all our commands
        CommandScheduler.getInstance().schedule(auto_commands);
    }
    public Command pause(double seconds){
        return new WaitUntil(time + seconds);
    }

    class WaitUntil extends CommandBase{
        double endTime;
        public WaitUntil(double endTime){
            this.endTime = endTime;
        }
        public boolean isFinished(){
            return(time > endTime);
        }
    }

    @Override
    public void loop() {
        drive.read_sensors(time);
        shooter.read_sensors(time);

        // Run the CommandScheduler instance
        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Elapsed time", runtime.toString());
        pack.put("time", time);
        pack.put("battery", battery.getVoltage());
        drive.add_telemetry(pack);
        shooter.add_telemetry(pack, telemetry);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // FIXME TODO put into FTC Dashboard too, for most of this
        Pose2D drivePosition = drive.getPosition();
        telemetry.addData("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", drivePosition.getX(DistanceUnit.METER), drivePosition.getY(DistanceUnit.METER), drivePosition.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.METER));
        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.METER));
        editor.apply();
        drive.stop();
        shooter.stop();
    }
}
