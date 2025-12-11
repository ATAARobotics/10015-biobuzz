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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Spindexer extends SubsystemBase {
    public MotorEx spindexerMotor;
    public NormalizedColorSensor brushland;

    // stuff we read from sensors
    double currentAngle;    // computed from our encoder
    double artifactDistance;  // raw distance guess from the sensor
    NormalizedRGBA artifactColor;  // raw colour from the sensor

    // stuff we derive
    boolean haveArtifact = false;
    public enum SpinDirection {Shoot, Index}
    SpinDirection spin = SpinDirection.Index;
    Timing.Timer stuckTime;

    // different PID tunings for "shoot" versus normal spin
    public PIDController storeControl;
    public PIDController shootControl;

    double spindexerPower;
    public int targetAngle;  // "no-reset" op-modes remember this targetAngle over auto->teleop transition

    // keeping track of which "slots" we have balls in.
    // slot 0 is the "forward" slot when we started (angle = 0)
    // slot 1 is the next one after that if we spindex "backwards" / non-shoot direction (so 1 position CCW from slot 0)
    // slot 2 is the next slot CCW from slot 1 (aka the slot 120degrees CW from slot 0)
    public enum SlotContent {Nothing, Purple, Green};
    SlotContent[] slots;  // this always has 3 elements: 0, 1 and 2

    public static double TOLERENCE_DEG = 10.0;
    public static double STEP_DEG = 120;
    // tuned December 10 with latest hardware rev (target collar, ramps, etc)
    public static PIDCoefficients shootPid = new PIDCoefficients(0.006, 0.02, 0.0003);
    public static PIDCoefficients storePid = new PIDCoefficients(0.005, 0.00, 0.0003);

    public Spindexer (HardwareMap hardwareMap) {
        spindexerMotor = new MotorEx(hardwareMap, "spindexer", Motor.GoBILDA.RPM_312);

        storeControl = new PIDController(storePid.p, storePid.i, storePid.d);
        shootControl = new PIDController(shootPid.p, storePid.i, storePid.d);
        storeControl.setTolerance(TOLERENCE_DEG);
        shootControl.setTolerance(TOLERENCE_DEG);

        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stuckTime = new Timing.Timer(600, TimeUnit.MILLISECONDS);
        brushland = hardwareMap.get(NormalizedColorSensor.class, "brushland");
        // we always have 3 slots in this array
        slots = new SlotContent[]{
                SlotContent.Nothing,
                SlotContent.Nothing,
                SlotContent.Nothing
        };
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

    // returns the index of the slot that's at the front of the robot;
    // at start this is slot 0
    public int currentSlot() {
        // we go from "targetAngle" because these are always whole
        // numbers .. so it'll be "a lie" until we're "at" a slot
        int norm = targetAngle;
        if (norm < 0) norm = -norm;
        norm = norm % 360;
        // integer division by 120 means anything less than 120 will
        // be 0, anything between 120-240 will be 1, anything from 240
        // to 360 will be 2
        return norm / 120;
    }

    public boolean isStuck(){
        if (stuckTime.done()){
            double diff = Math.abs(targetAngle - currentAngle);
            if (diff > 10){
                return true;
            }
        }
        return false;
    }

    public boolean atTarget(){
        if (spin == SpinDirection.Index) {
            return storeControl.atSetPoint();
        } else {
            return shootControl.atSetPoint();
        }
    }

    public void read_sensors(double time) {
        currentAngle = ticksToDeg(spindexerMotor.getCurrentPosition());
        artifactColor = brushland.getNormalizedColors();
        artifactDistance = ((OpticalDistanceSensor)brushland).getLightDetected();
    }

    @Override
    public void periodic() {
        // prelim tests show that we get like 0.0xxx values with
        // nothing, and 0.25 to 0.30 ish values when there's a ball
        // (but NOT when a hole is rotated there)
        // ...also we don't want to try detections when we're "between" slots
        haveArtifact = false;
        if (atTarget()) {
            haveArtifact = artifactDistance > 0.19;
            if (haveArtifact) {
                if (slots[currentSlot()] == SlotContent.Nothing) {
                    slots[currentSlot()] = SlotContent.Purple;
                    //targetAngle -= STEP_DEG;
                }
            }
        }

//        storeControl.setPID(storePid.p, storePid.i, storePid.d);
//        shootControl.setPID(shootPid.p, shootPid.i, shootPid.d);

        // for CCW ("shoot") direction, we use a separate PID .. so we
        // ned to know "which direction" we're spinning.
        PIDController control = spin == SpinDirection.Index ? storeControl : shootControl;
        spindexerPower = control.calculate(currentAngle - targetAngle);
        //if (spindexerPower > 0.5) spindexerPower = 0.5;
        //if (spindexerPower < -0.5) spindexerPower = -0.5;
        spindexerMotor.set(spindexerPower);
    }

    private String renderSlot(int i) {
        String s = "[ ";
        if (slots[i] == SlotContent.Nothing) s += "  ]";
        if (slots[i] == SlotContent.Purple) s +=  "P ]";
        if (slots[i] == SlotContent.Green) s += "G ]";
        return s;
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("spindexer-target-angle", targetAngle);
        telem.log("spindexer-current-angle", currentAngle);
        telem.log("spindexer-have-artifact", haveArtifact);
        telem.log("spindexer-artifact-red", artifactColor.red);
        telem.log("spindexer-artifact-green", artifactColor.green);
        telem.log("spindexer-artifact-blue", artifactColor.blue);
        telem.log("spindexer-artifact-distance", artifactDistance);
        telem.log("spindexer-current-slot", currentSlot());
        telem.log("spindexer-at-target", atTarget());
        telem.log("spindexer-power", spindexerPower);
        telem.log("spindexer-stuck", isStuck());

        telem.logBoth("Loaded", renderSlot(0) + " " + renderSlot(1) + " " + renderSlot(2));    }

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
                    spin = SpinDirection.Shoot;
                    shootControl.reset();
                    targetAngle += STEP_DEG;
                    stuckTime.start();
                }
                else {
                    operator.gamepad.rumble(100);
                }
            }
            if (operator.wasJustPressed(GamepadKeys.Button.B)) {
                storeControl.setPID(storePid.p, storePid.i, storePid.d);
                shootControl.setPID(shootPid.p, shootPid.i, shootPid.d);
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)){
                spin = SpinDirection.Index;
                storeControl.reset();
                targetAngle -= STEP_DEG;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.X)){
                spin = SpinDirection.Shoot;
                shootControl.reset();
                targetAngle += (3 * STEP_DEG);
                stuckTime.start();
            }

        }
    }
}
