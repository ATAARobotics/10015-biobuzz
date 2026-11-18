package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Transfer extends SubsystemBase {
    HyperTelemetry telem;
    private MotorEx intakeMotor;
    private MotorEx transferMotor;
    private Servo pollenBlocker;
    public static double POLLEN_SHOOT = 0.0;
    public static double POLLEN_BLOCK = 0.16; // nov 17

    // adjusted november 17
    public static double TRANSFER_TIME = 1.2;  // seconds
    public static double MIN_TRANSFER = 0.65;
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

    public Transfer(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
         intakeMotor = new MotorEx(hardwareMap, "intake");//port 0 expansion
         intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
           transferMotor = new MotorEx(hardwareMap, "transfer");//port 1 expansion
            transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // magazineServo = hardwareMap.get(Servo.class, "magazine"); Commented out bc we don't need until next comp
          pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker"); // port 5 control hub
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker"); Commented out bc we don't need until next comp
        crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");
        crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");
        // beam break is analog 0 control hub
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
    

    // set motors and servos properly in here (only)
    @Override
    public void periodic() {
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
        double percentTransfer = (time - shootStartTime) / TRANSFER_TIME;
        if (percentTransfer < 0.0) percentTransfer = 0.0;
        if (percentTransfer > 1.0) percentTransfer = 1.0;

        switch (transferMode) {
        case IDLE:
            break;
        case SHOOT:
            transferPower = MIN_TRANSFER + (percentTransfer * (MAX_TRANSFER - MIN_TRANSFER));
            break;
        case REJECT:
            transferPower = -MIN_TRANSFER;
            break;
        }
        telem.log("transfer-power", transferPower);
        transferMotor.set(transferPower);

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
    }

    public class HumanInputs extends CommandBase {
        GamepadEx operator;

        public HumanInputs(GamepadEx operator) {
            this.operator = operator;
            addRequirements(Transfer.this);
        }

        @Override
        public void execute() {
          /* if (operator.getLeftY() > 0.1){
                runIntake();
            } else if (operator.getLeftY() < -0.1){
                reverseIntake();
            } else{
                 stopIntake();
             }*/
        }
    }
}
