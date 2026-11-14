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
    //TODO: Set values for crash bar positions
    private final double crashOneNothing = -1;
    private final double crashOneEverything = -1;
    private final double crashOnePollen = -1;
    private final double crashTwoNothing = -1;
    private final double crashTwoEverything = -1;
    private final double crashTwoPollen = -1; //all set to -1 so the code crashes instead of wrecking robot with guessed numbers
    public enum CrashBarMode{NOTHING, EVERYTHING, POLLEN}
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
    public void reverseIntakeMotor(){
        intakeMotor.set(-intakePower);
    }
    public void runTransferMotor(){
        transferMotor.set(transferPower);
    }
    public void stopTransferMotor(){
        transferMotor.set(0);
    }
    public void setCrashBar(CrashBarMode newMode){
        if(newMode == CrashBarMode.NOTHING){
            crashBarServo1.setPosition(crashOneNothing);
            crashBarServo2.setPosition(crashTwoNothing);
        }
        else if(newMode == CrashBarMode.EVERYTHING){
            crashBarServo1.setPosition(crashOneEverything);
            crashBarServo2.setPosition(crashTwoEverything);
        }
        else if(newMode == CrashBarMode.POLLEN){
            crashBarServo1.setPosition(crashOnePollen);
            crashBarServo2.setPosition(crashTwoPollen);
        }
    }
    public void pollenBlockerOpen(){

    }
    public void pollenBlockerClosed(){

    }
}
