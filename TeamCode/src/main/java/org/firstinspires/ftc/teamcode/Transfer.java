package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Transfer extends SubsystemBase {
    HyperTelemetry telem;
    private MotorEx intakeMotor;
    private MotorEx transferMotor;
    private  Servo pollenBlocker;
    private   Servo magazineServo;
    private  Servo crashBarServo1;
    private Servo crashBarServo2;
    private Servo nectarBlocker;
    public enum IntakeMode{IN, OUT, IDLE}
    private IntakeMode mode = IntakeMode.IDLE;
    private double intakePower = 1;
    private double transferPower = 0.4;

    public Transfer(HardwareMap hardwareMap, HyperTelemetry telem){
        this.telem = telem;
        intakeMotor = new MotorEx(hardwareMap, "intake");
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor = new MotorEx(hardwareMap, "transfer");
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       // magazineServo = hardwareMap.get(Servo.class, "magazine"); Commented out bc we don't need until next comp
        pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker");
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker"); Commented out bc we don't need until next comp
        crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");
        crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");
    }

    public void runIntakeMotor(){
        intakeMotor.set(intakePower);
    }
    public void stopIntakeMotor(){
        intakeMotor.set(0);
    }
    public void runTransferMotor(){
        transferMotor.set(transferPower);
    }
    public void stopTransferMotor(){
        transferMotor.set(0);
    }
    public void setCrashBarMode(){

    }
    public void pollenBlockerOpen(){

    }
    public void pollenBlockerClosed(){

    }
}
