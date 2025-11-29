package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake extends SubsystemBase {
    private MotorEx intake;
    public enum IntakeMode { In, Out, Idle }
    private IntakeMode mode = IntakeMode.Idle;
    public Intake (HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, "intake");
    }
    public void reset(){
        mode = IntakeMode.Idle;
    }
    public void grab(){
        mode = IntakeMode.In;
    }
    public void stop(){
        mode = IntakeMode.Idle;
    }

    public void spit(){
        mode = IntakeMode.Out;
    }

    @Override
    public void periodic() {
        if (mode == IntakeMode.In) {
            intake.set(1.0);
        } else if (mode == IntakeMode.Out) {
            intake.set(-0.5);
        } else {
            intake.set(0.0);
        }
    }

    public Command takeIn() {
        return new TakeIn();
    }

    public class TakeIn extends CommandBase {
        @Override
        public void initialize() {
            grab();
        }
        public boolean isFinished(boolean interrupted) {
            return true;
        }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("intake-mode", mode);
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
            if (operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                mode = IntakeMode.In;
            } else if (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                mode = IntakeMode.Out;
            } else {
                mode = IntakeMode.Idle;
            }
        }
    }

}
