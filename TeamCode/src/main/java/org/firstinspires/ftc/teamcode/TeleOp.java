package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.List;

@Configurable
public abstract class TeleOp extends RobotBaseOp {

    protected void bindDriverControls() {
        // auto intake mode
        TriggerHeld driverRight = new TriggerHeld(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        driverRight.whileActiveOnce(new AutoIntake(), true);
        // auto outtake mode
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new AutoOuttake(), true);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new OuttakeOff(), true);
    }

    protected void bindOperatorControls() {
        // spindexer
        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            spindexer.new ShootOnce()
            );
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            new SequentialCommandGroup(
                spindexer.new ShootOnce(),
                spindexer.new ShootOnce(),
                spindexer.new ShootOnce()
                )
            );
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            spindexer.new IndexOnce()
        );
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ToggleShoot()
        );
    }
}
