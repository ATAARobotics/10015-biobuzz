package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Spindexer extends SubsystemBase {
    public MotorEx spindexerMotor;
    public PIDController spindexerPID;
    double targetAngle;
    double currentAngle;
    public static double TOLERENCE_DEG;
    public static double STEP_DEG = 120;
    public static double TICKS_PER_REV = 537.7;
    public static double GEAR_RATIO = 19.2;
    public static double spindexerP = 0.0, spindexerI = 0.0, spindexerD = 0.0;
    public Spindexer (HardwareMap hardwareMap){
        spindexerMotor = new MotorEx(hardwareMap, "spindexerMotor", Motor.GoBILDA.RPM_312);
        spindexerPID = new PIDController(spindexerP,spindexerI,spindexerD);
        spindexerPID.setTolerance(TOLERENCE_DEG);
        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        return angle;
    }
    public void add_telemetry(TelemetryPacket pack, Telemetry telemetry) {
        telemetry.addData("targetAngle", targetAngle);
        telemetry.addData("currentAngle", currentAngle);

        pack.put("spindexer P", spindexerP);
        pack.put("spindexer I", spindexerI);
        pack.put("spindexer D", spindexerD);
    }
    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        GamepadEx operator;

        public HumanInputs(GamepadEx operator, GamepadEx driver) {
            this.operator = operator;
            this.driver = driver;
            addRequirements(Spindexer.this);
        }

        @Override
        public void execute() {
            if (operator.isDown(GamepadKeys.Button.A)){
                spindexerMotor.set(0.2);
            }
            else {
                spindexerMotor.set(0);
            }

        }
    }
}
