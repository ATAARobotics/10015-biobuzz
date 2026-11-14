package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandScheduler;
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
    }

    protected void bindOperatorControls() {
        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SetCrashBarNothing(), true);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SetCrashBarPollen(), true);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new RPMUp(), true);
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new RPMDown(), true);
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ToggleRPM(), true);
    }
}
