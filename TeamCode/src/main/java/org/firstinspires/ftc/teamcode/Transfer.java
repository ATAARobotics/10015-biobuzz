package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import java.util.LinkedList;

@Configurable
public class Transfer extends SubsystemBase {
    HyperTelemetry telem;
    private MotorEx intakeMotor;
    private MotorEx transferMotor;
    private Servo pollenBlocker;
    public static double POLLEN_SHOOT = 0.0;
    public static double POLLEN_BLOCK = 0.18; // nov 17

    // adjusted november 17
    public static double TRANSFER_TIME = 1.0;  // seconds
    public static double RAMP_START = 0.70;
    public static double MIN_TRANSFER = 0.66;
    public static double MAX_TRANSFER = 1.0;
    
    public enum IntakeMode {IDLE, IN, OUT}
    public enum TransferMode {IDLE, SHOOT, REJECT}
    public enum CrashBarMode {START, NECTAR, POLLEN, NOTHING}

    private Servo magazineServo;
    private Servo crashBarServo1;//port 0 control hub
    private Servo crashBarServo2;//port 5 expansion hub
    private Servo nectarBlocker;

    // state of sensors we read
    private double time;

    // the state of things we _want_ which will be set in periodic
    private boolean shooting = false;
    private double accelStartTime;
    private IntakeMode intakeMode = IntakeMode.IDLE;
    private TransferMode transferMode = TransferMode.IDLE;
    private CrashBarMode crashMode = CrashBarMode.START;
    private double currentIntakePower = 0.0;
    private double shootStartTime = 0.0;
    
    //The value of the two crash bars for each position MUST ADD UP TO ONE.
    private final double crashOneStart = 0;
    private final double crashTwoStart = 1;
    private final double crashOneNothing = 0.5;
    private final double crashTwoNothing = 0.5;
    private final double crashOneNectar = 0.85;
    private final double crashTwoNectar = 0.15;
    private final double crashOnePollen = 1;
    private final double crashTwoPollen = 0;
    AnalogInput beamBreak;
    double beamBreakVoltage;
    public static double BEAM_BLOCKED_VOLTS = 1;
    boolean blocked;
    int blockedSamples = 0;
    int clearSamples = 0;
    int DEBOUNCE_SAMPLES = 3;
    boolean magazineFull = false;

    public Transfer(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
        intakeMotor = new MotorEx(hardwareMap, "intake");//port 0 expansion
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor = new MotorEx(hardwareMap, "transfer");//port 1 expansion
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // magazineServo = hardwareMap.get(Servo.class, "magazine"); Commented out bc we don't need until next comp
        pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker"); // port 5 control hub
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker"); Commented out bc we don't need until next comp
        crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");// 0 on control
        crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");//five on expansion
        beamBreak = hardwareMap.analogInput.get("beam_break"); // beam break is analog 3 control hub

    }

    public void readSensors(double time) {
        this.time = time;
    }

    public void runIntake(){
        intakeMode = IntakeMode.IN;
    }

    public void reverseIntake(){
        intakeMode = IntakeMode.OUT;
    }
    
    public void stopIntake(){
        intakeMode = IntakeMode.IDLE;
    }

    public void runTransfer(){
        transferMode = TransferMode.SHOOT;
    }
    
    public void reverseTransfer(){
        transferMode = TransferMode.REJECT;
    }

    public void stopTransfer(){
        transferMode = TransferMode.IDLE;
    }

    public void setCrashBarNothing() {
        crashMode = CrashBarMode.NOTHING;
    }
    //       crashBarServo1.setPosition(crashOneNothing);
    //      crashBarServo2.setPosition(crashTwoNothing);

    public void setCrashBarNectar() {
        crashMode = CrashBarMode.NECTAR;
    }
    //crashBarServo1.setPosition(crashOneNectar);
    //crashBarServo2.setPosition(crashTwoNectar);

    public void setCrashBarPollen() {
        crashMode = CrashBarMode.POLLEN;
    }

    //crashBarServo1.setPosition(crashOnePollen);
    //crashBarServo2.setPosition(crashTwoPollen);

    public void setCrashBarStart() {
        crashMode = CrashBarMode.START;
    }

    //crashBarServo1.setPosition(crashOneStart);
    //crashBarServo2.setPosition(crashTwoStart);

    public void togglePollenBlocker() {
        shooting = !shooting;
    }
    
    public void pollenBlocked() {
        shooting = false;
    }
    
    public void pollenUnblocked(){
        shooting = true;
    }

    public void shoot() {
        shootStartTime = time;
        runTransfer();
        runIntake();
        pollenUnblocked();
    }

    // opposite of "shoot"
    public void idle() {
        stopTransfer();
        stopIntake();
        pollenBlocked();
    }
    public void updateBeamBreak(){
        beamBreakVoltage = (beamBreak.getVoltage());
        blocked = beamBreakVoltage < BEAM_BLOCKED_VOLTS;
        if (blocked){
            blockedSamples ++;
            clearSamples = 0;
            if (blockedSamples > DEBOUNCE_SAMPLES){
                magazineFull = true;
            }
        }
        else{
            clearSamples ++;
            blockedSamples = 0;
            if(clearSamples > DEBOUNCE_SAMPLES) {
                magazineFull = false;
            }

        }
    }
    public boolean isMagazineFull(){
        return magazineFull;
    }
    

    // set motors and servos properly in here (only)
    @Override
    public void periodic() {
        updateBeamBreak();
        // shot-blocker servo
        if (shooting) {
            pollenBlocker.setPosition(POLLEN_SHOOT);
        } else {
            pollenBlocker.setPosition(POLLEN_BLOCK);
        }

        // intake motor
        double intakePower = 0.0;
        switch (intakeMode) {
        case IDLE:
            break;
        case IN:
            intakePower = -0.9;
            break;
        case OUT:
            intakePower = 0.9;
            break;
        }
        intakeMotor.set(intakePower);

        // transfer
        double transferPower = 0.0;
        // figure out how far we are between start-time and end-time of transfer
	double elapsed = time - shootStartTime;

        switch (transferMode) {
        case IDLE:
            break;
        case SHOOT:
	    if (elapsed < RAMP_START) {
		transferPower = MIN_TRANSFER;
	    } else {
		// what % we are through the ramp -> end
		double percentTransfer = (elapsed - RAMP_START) / (TRANSFER_TIME - RAMP_START);
		if (percentTransfer < 0.0) percentTransfer = 0.0;
		if (percentTransfer > 1.0) percentTransfer = 1.0;
		transferPower = MIN_TRANSFER + (percentTransfer * (MAX_TRANSFER - MIN_TRANSFER));
	    }
            break;
        case REJECT:
            transferPower = -MIN_TRANSFER;
            break;
        }
        transferMotor.set(transferPower);

        if (intakeMode == IntakeMode.IN && isMagazineFull()){
            crashMode = CrashBarMode.NOTHING;

        }
        // crash-bar servos. IMPORTANT: the two values should sum to 1!
        switch (crashMode) {
        case START:
            crashBarServo1.setPosition(crashOneStart);
            crashBarServo2.setPosition(crashTwoStart);
            break;
        case NECTAR:
            crashBarServo1.setPosition(crashOneNectar);
            crashBarServo2.setPosition(crashTwoNectar);
            break;
        case POLLEN:
            crashBarServo1.setPosition(crashOnePollen);
            crashBarServo2.setPosition(crashTwoPollen);
            break;
        case NOTHING:
            crashBarServo1.setPosition(crashOneNothing);
            crashBarServo2.setPosition(crashTwoNothing);
            break;
        }

        // telem last
        telem.log("transfer-power", transferPower);
        telem.log("transfer-beambreak", beamBreakVoltage);
        telem.log("transfer-full", blocked);
        telem.log("crash-bar-mode", crashMode);
    }
}
