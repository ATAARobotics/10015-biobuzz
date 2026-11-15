package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class TeleOp extends RobotBaseOp {
    @Override
    public void start(){
        super.start();
        OpmodeStorage.autoRan=false;
    }
    public boolean isAuto(){
        return false;
    }

    protected void bindDriverControls() {
      //  CommandScheduler.getInstance().setDefaultCommand(shooter, shooter.new HumanInputs(operator, driver));
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
        TriggerHeld driverLeft = new TriggerHeld(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        driverLeft.whenActive(new StartAutoIntake(), true);
        driverLeft.whenInactive(new StopAutoIntake(), true);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed((new AutoShoot()));
    }

    protected void bindOperatorControls() {
        TriggerHeld operatorLeft = new TriggerHeld(operator, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerHeld operatorRight = new TriggerHeld(operator, GamepadKeys.Trigger.RIGHT_TRIGGER);
        operatorRight.whenActive(new InstantCommand(transfer::runTransfer, transfer));
        operatorLeft.whenActive(new InstantCommand(transfer::reverseTransfer, transfer));
        operatorRight.whenInactive(new InstantCommand(transfer::stopTransfer, transfer));
        operatorLeft.whenInactive(new InstantCommand(transfer::stopTransfer, transfer));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(transfer::setCrashBarNothing, transfer));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(transfer::setCrashBarPollen, transfer));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(shooter::rpmUp));
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(shooter::rpmDown));
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(shooter::toggleRpm, shooter));
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(transfer::togglePollenBlocker));

        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(transfer::setCrashBarStart, transfer));//just for sizing
        CommandScheduler.getInstance().setDefaultCommand(transfer, transfer.new HumanInputs(operator));
    }
}
