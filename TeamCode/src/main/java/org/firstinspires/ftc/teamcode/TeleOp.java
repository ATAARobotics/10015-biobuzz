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

        // "mostly" we want to run the HumanInputs commands during teleop
        CommandScheduler.getInstance().setDefaultCommand(drive, drive.new HumanInputs(driver));
    }

    protected void bindOperatorControls() {
      //  operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(shooter.alwaysSpin());
    }
}
