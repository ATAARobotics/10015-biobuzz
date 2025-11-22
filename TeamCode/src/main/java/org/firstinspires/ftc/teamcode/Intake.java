package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake extends SubsystemBase {
    public MotorEx intake;

    public Intake (HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, "intake");
    }

    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        GamepadEx operator;

        public HumanInputs(GamepadEx operator, GamepadEx driver) {
            this.operator = operator;
            this.driver = driver;
            addRequirements(Intake.this);
        }

        @Override
        public void execute() {
            // decide what to do based on sensors and human inputs from controller

            // todo: ideally we'd set "what the user wants to do" and
            // only during "periodic" would we actually call motor
            // commands like .set()
            intake.set(operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                intake.set(-operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) / 0.5);
            }

        }
    }

}
