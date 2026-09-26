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
    int loops;

    Drive drive;
    double strafe;
    double forward;
    double turn;

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
        telem.log("elapsed", runtime.seconds());
        telem.log("time", time);
        telem.log("battery", battery.getVoltage());
        telem.log("alliance", getAlliance());

        double fps = loops / runtime.seconds();
        telem.logDrivers("average fps", fps);

        drive.addTelemetry(telem);

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
