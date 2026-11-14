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
    public static double pollenShooting = 0.05;
    public static double pollenBlocked = 0.2;
    private boolean shooting = false;
    private Servo magazineServo;
    private Servo crashBarServo1;//port 0 control hub
    private Servo crashBarServo2;//port 5 expansion hub
    private Servo nectarBlocker;

    public enum IntakeMode {IN, OUT, IDLE}

    private IntakeMode mode = IntakeMode.IDLE;
    private double intakePower = 1;
    private double transferPower = 1;//0.29;
    //The value of the two crash bars for each position MUST ADD UP TO ONE.
    private final double crashOneStart = 0;
    private final double crashTwoStart = 1;
    private final double crashOneNothing = 0.5;
    private final double crashTwoNothing = 0.5;
    private final double crashOneNectar = 0.85;
    private final double crashTwoNectar = 0.15;
    private final double crashOnePollen = 1;
    private final double crashTwoPollen = 0;

    public enum CrashBarMode {NOTHING, NECTAR, POLLEN, START}

    CrashBarMode crashBarMode;

    public Transfer(HardwareMap hardwareMap, HyperTelemetry telem) {
        this.telem = telem;
         intakeMotor = new MotorEx(hardwareMap, "intake");//port 0 expansion
         intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
           transferMotor = new MotorEx(hardwareMap, "transfer");//port 1 expansion
            transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // magazineServo = hardwareMap.get(Servo.class, "magazine"); Commented out bc we don't need until next comp
          pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker");
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker"); Commented out bc we don't need until next comp
        crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");
        crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");
    }

      public void runIntake(){
          intakeMotor.set(intakePower);
      }
      public void reverseIntake(){
          intakeMotor.set(-intakePower);
      }
    public void stopIntake(){
        intakeMotor.set(0);
    }
      public void runTransfer(){
          transferMotor.set(transferPower);
      }
      public void reverseTransfer(){
          transferMotor.set(-transferPower);
      }
      public void stopTransfer(){
        transferMotor.set(0);
      }
    public void setCrashBarNothing() {
            crashBarServo1.setPosition(crashOneNothing);
            crashBarServo2.setPosition(crashTwoNothing);
    }

    public void setCrashBarNectar() {
            crashBarServo1.setPosition(crashOneNectar);
            crashBarServo2.setPosition(crashTwoNectar);
    }
        public void setCrashBarPollen() {
                crashBarServo1.setPosition(crashOnePollen);
                crashBarServo2.setPosition(crashTwoPollen);

        }
        public void setCrashBarStart() {
                crashBarServo1.setPosition(crashOneStart);
                crashBarServo2.setPosition(crashTwoStart);
        }

        public void togglePollenBlocker(){
            if (shooting){
                pollenBlocker.setPosition(pollenBlocked);
                shooting = false;
            }
            else{
                pollenBlocker.setPosition(pollenShooting);
                shooting = true;
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
            if (operator.getLeftY() > 0.1){
                runIntake();
            }
            else if (operator.getLeftY() < -0.1){
                reverseIntake();
            }
        }
    }
}