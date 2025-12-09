package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

@Config
public class Spindexer extends SubsystemBase {
    public MotorEx spindexerMotor;
    Timing.Timer stuckTime;
    public PIDController spindexerPID;
    double spindexerPower;
    public double targetAngle;  // "no-reset" op-modes remember this targetAngle over auto->teleop transition
    double currentAngle;    // computed from our encoder
    public static double TOLERENCE_DEG = 1.0;
    public static double STEP_DEG = 120;
    // tuned november 24 end of session
    public static double spindexerP = 0.026, spindexerI = 0.01, spindexerD = 0.0005;
    public Spindexer (HardwareMap hardwareMap){
        spindexerMotor = new MotorEx(hardwareMap, "spindexer", Motor.GoBILDA.RPM_312);
        spindexerPID = new PIDController(spindexerP,spindexerI,spindexerD);
        spindexerPID.setTolerance(TOLERENCE_DEG);
        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stuckTime = new Timing.Timer(600, TimeUnit.MILLISECONDS);

    }

    public void reset() {
        currentAngle = 0;
        targetAngle = 0;
        spindexerMotor.set(0);
        // drive-team needs to orient Spindexer with one segment forward
        spindexerMotor.resetEncoder();
    }

    private double ticksToDeg(int ticks){
       double motorRevs = ticks/spindexerMotor.getCPR();
       return motorRevs * 360;
    }
    public void spinccw(){
        targetAngle += STEP_DEG;
    }

    public boolean isStuck(){
        if (stuckTime.done()){
            double diff = targetAngle - currentAngle;
            if (diff > 10){
                return true;
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        currentAngle = ticksToDeg(spindexerMotor.getCurrentPosition());
        spindexerPID.setPID(spindexerP, spindexerI, spindexerD);
        spindexerPower = spindexerPID.calculate(currentAngle - targetAngle);
       // if (spindexerPower > 0.5) spindexerPower = 0.5;
        if (spindexerPower < -0.5) spindexerPower = -0.5;
        spindexerMotor.set(spindexerPower);
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.logBoth("spindex-target-angle", targetAngle);
        telem.logBoth("spindex-current-angle", currentAngle);

        telem.log("spindexer-p", spindexerP);
        telem.log("spindexer-i", spindexerI);
        telem.log("spindexer-d", spindexerD);
        telem.log("spindexerPower", spindexerPower);
        telem.log("spindexer-stuck", isStuck());
    }

// TODO: there's a nicer way to do this, which may be more reusable in Auto
// we can bind buttons / etc to "run commands"
// e.g.:
//     operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SpindexCCW());
//
// we must take care to think about when new commands will run, what gets "taken over", etc
// (remember: one subsystem may only run one command at a time).
// figure out: does e.g. a subsequent "Y" press "override" the command? e.g. cancel the previous?
//
// this could be better for "spindex" e.g. because we could have a
// "timeout" (and maybe even "cancel" back to the previous target
// angle?)

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
            if (operator.wasJustPressed(GamepadKeys.Button.A)){
                double dif = targetAngle - currentAngle;
                if (dif < 10) {
                    targetAngle += STEP_DEG;
                    stuckTime.start();
                }
                else {
                    operator.gamepad.rumble(100);

                }
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)){
                targetAngle -= STEP_DEG;
            }

        }
    }
}
