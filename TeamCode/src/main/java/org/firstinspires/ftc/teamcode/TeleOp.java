package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class TeleOp extends RobotBaseOp {

    public boolean isAuto(){
        return false;
    }

    protected void bindDriverControls() {
        // auto intake mode
        TriggerHeld driverRight = new TriggerHeld(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        driverRight.whileActiveOnce(new AutoIntake(), true);
        // auto outtake mode
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new AutoOuttake(), true);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new OuttakeOff(), true);

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
        CommandScheduler.getInstance().setDefaultCommand(shooter, shooter.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(turret, turret.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(intake, intake.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(spindexer, spindexer.new HumanInputs(operator, driver));
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
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new OperatorOffset()
        );
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new PrepareToShoot()
        );
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new UnShoot()
        );
    }
}
