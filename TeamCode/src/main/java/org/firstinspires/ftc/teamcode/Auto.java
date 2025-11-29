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
    Turret turret;
    Shooter shooter;
    Spindexer spindexer;
    Intake intake;
    GamepadEx driver;
    GamepadEx operator;
    double pauseTime = 0;
    int PAUSE_TIME_INCREMENT = 1;
    ElapsedTime runtime = new ElapsedTime();

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    //VisionPortal portal;

    public enum Alliance {RED, BLUE}
    public enum AutoStartPos {FAR, NEAR}
    public abstract AutoStartPos getStartPos();
    boolean isFar;
    public abstract Alliance getAlliance();
    boolean isRedAlliance;
    boolean parkingDefault = true;
    AprilTagMetadata target;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        isRedAlliance = getAlliance() == Alliance.RED;
        isFar = getStartPos() == AutoStartPos.FAR;

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

       /* portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"DubbleBubble Webcam"))
                .addProcessor(april_tags)
                .setCameraResolution(new Size(640, 480))
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
*/
        // todo: a lot of this repeats in Auto and TeleOp -- can we combine?
        drive = new Drive(hardwareMap, isRedAlliance);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        battery = hardwareMap.voltageSensor.get("Control Hub");

        // Register Subsystem objects to the scheduler
        CommandScheduler.getInstance().registerSubsystem(drive);
        CommandScheduler.getInstance().registerSubsystem(shooter);
        CommandScheduler.getInstance().registerSubsystem(turret);
        CommandScheduler.getInstance().registerSubsystem(intake);
        CommandScheduler.getInstance().registerSubsystem(spindexer);

        drive.reset();
        shooter.reset();
        spindexer.reset();
        intake.reset();
    }

    @Override
    public void init_loop() {
        driver.readButtons();
        operator.readButtons();
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            pauseTime += PAUSE_TIME_INCREMENT;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            pauseTime -= PAUSE_TIME_INCREMENT;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {parkingDefault = true;}
        if (operator.wasJustPressed((GamepadKeys.Button.RIGHT_BUMPER))) {parkingDefault = false;}

        if (pauseTime < 0){
            pauseTime = 0;
        }
        telemetry.addData("Pause Time: ", pauseTime);
        telemetry.addData("Alliance: ", getAlliance());
        telemetry.addData("Starting Position: ", getStartPos());
        telemetry.addData("Parking Position: ", parkingDefault ? "Default" : "Opt. 2");
        telemetry.update();
    }

    // origin is center of the field, in meters
    // Looking from audience side, axis X is pointing left and axis Y is pointint forward
    @Override
    public void start() {
        runtime.reset();
        turret.reset();
        spindexer.reset();
        intake.reset();
        // OPTION 1: starting position is touching audience field perimeter wall
//        drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 0.381 : -0.381, -1.556, AngleUnit.DEGREES, 180));

        // OPTION 2: starting position is over the center of a launch line touching own alliance's goal:
        if (isFar){
            drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 0.404 : -0.404, -1.552, AngleUnit.DEGREES, 0));
            SequentialCommandGroup auto_commands = new SequentialCommandGroup(
                    pause(pauseTime),
                    drive.moveQuickly(isRedAlliance ? 0.404 : -0.404, -1.390, isRedAlliance ? -22 : 22).withTimeout(1500),
                    shooter.shoot(spindexer, intake),
                    shooter.shoot(spindexer, intake),
                    shooter.shoot(spindexer, intake)
            );
            if (parkingDefault) {
                auto_commands.addCommands(
                        drive.moveQuickly(isRedAlliance ? 0.404 : -0.404, -1.182, isRedAlliance ? -90 : 90)
                );
            }
            CommandScheduler.getInstance().schedule(auto_commands);
        } else {
            drive.setPosition(new Pose2D(DistanceUnit.METER, isRedAlliance ? 1.191 : -1.191, 1.457, AngleUnit.DEGREES, isRedAlliance ? 135 : -135));
            SequentialCommandGroup auto_commands = new SequentialCommandGroup(
                    pause(pauseTime),
                    drive.moveQuickly(isRedAlliance ? 1.0 : -1.0, 1.0, isRedAlliance ? 135 : -135).withTimeout(2500),
                    shooter.shoot(spindexer, intake),
                    shooter.shoot(spindexer, intake),
                    shooter.shoot(spindexer, intake)
            );
            if (parkingDefault) {
                auto_commands.addCommands(
                        drive.moveQuickly(isRedAlliance ? 0.381 : -0.381, 1.4, 90)
                );
                CommandScheduler.getInstance().schedule(auto_commands);
            }
        }

        // schedule all our commands
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
        HyperTelemetry telem = new HyperTelemetry(telemetry, pack);
        telem.log("elapsed", runtime.toString());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());

        drive.addTelemetry(telem);
        shooter.addTelemetry(telem);
        turret.addTelemetry(telem);
        intake.addTelemetry(telem);
        spindexer.addTelemetry(telem);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);

        // log some drivetrain information always too
        Pose2D drivePosition = drive.getPosition();
        double x = drivePosition.getX(DistanceUnit.METER);
        double y = drivePosition.getY(DistanceUnit.METER);
        double h = drivePosition.getHeading(AngleUnit.DEGREES);
        telem.log("position-x", x);
        telem.log("position-y", y);
        telem.log("position-heading", h);
        telem.logDrivers("Robot Position", "x = %4.2f, y = %4.2f, h = %4.2f", x, y, h);
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
        editor.putFloat("turret", (float)turret.getServoAngle());
        editor.apply();
        drive.stop();
        shooter.stop();
    }
}
