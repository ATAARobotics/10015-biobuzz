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
    private Servo magazineServo;
    private Servo crashBarServo1;//port 0 control hub
    private Servo crashBarServo2;//port 5 expansion hub
    private Servo nectarBlocker;

    public enum IntakeMode {IN, OUT, IDLE}

    private IntakeMode mode = IntakeMode.IDLE;
    private double intakePower = 1;
    private double transferPower = 0.4;
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
        // intakeMotor = new MotorEx(hardwareMap, "intake");
        // intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //   transferMotor = new MotorEx(hardwareMap, "transfer");
        //    transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // magazineServo = hardwareMap.get(Servo.class, "magazine"); Commented out bc we don't need until next comp
        //  pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker");
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker"); Commented out bc we don't need until next comp
        crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");
        crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");
    }

    /*  public void runIntakeMotor(){
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
      }*/
    public void setCrashBarNothing() {
        if (crashBarMode == CrashBarMode.NOTHING) {
            crashBarServo1.setPosition(crashOneNothing);
            crashBarServo2.setPosition(crashTwoNothing);
        }
    }

    public void setCrashBarNectar() {
        if (crashBarMode == CrashBarMode.NECTAR) {
            crashBarServo1.setPosition(crashOneNectar);
            crashBarServo2.setPosition(crashTwoNectar);
        }
    }
        public void setCrashBarPollen() {
            if (crashBarMode == CrashBarMode.POLLEN) {
                crashBarServo1.setPosition(crashOnePollen);
                crashBarServo2.setPosition(crashTwoPollen);
            }
        }
        public void setCrashBarStart() {
            if (crashBarMode == CrashBarMode.START) {
                crashBarServo1.setPosition(crashOneStart);
                crashBarServo2.setPosition(crashTwoStart);
            }
        }
        public void pollenBlockerOpen() {

        }
        public void pollenBlockerClosed() {

        }
        public class HumanInputs extends CommandBase {
            GamepadEx operator;

            public HumanInputs(GamepadEx operator) {
                this.operator = operator;
                addRequirements(Transfer.this);
            }

            @Override
            public void execute() {
                if (operator.wasJustPressed(GamepadKeys.Button.B)) {
                    setCrashBarNothing();
                }
                if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                    setCrashBarNectar();
                }
                if (operator.wasJustPressed(GamepadKeys.Button.X)) {
                    setCrashBarPollen();
                }
                if (operator.wasJustPressed(GamepadKeys.Button.A)) {
                    setCrashBarStart();
                }
            }
        }
    }