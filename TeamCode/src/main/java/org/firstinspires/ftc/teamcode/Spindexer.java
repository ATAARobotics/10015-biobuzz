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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Spindexer extends SubsystemBase {
    public MotorEx spindexerMotor;

    // stuff we read from sensors
    double currentAngle;    // computed from our encoder
    DigitalChannel artifact_color;
    DigitalChannel artifact_distance;
    AnalogInput analog;

    // stuff we derive
    public enum SpinDirection {Shoot, Index}
    SpinDirection spin = SpinDirection.Index;
    public enum Mode {Auto, Manual}
    Mode mode = Mode.Auto;
    double manualPower;
    Timing.Timer stuckTime;

    // different PID tunings for "shoot" versus normal spin
    public PIDController storeControl;
    public PIDController shootControl;
    public boolean boostF = false;
    public static double boostAmount = 0.5;//0.85;

    double spindexerPower;
    public int targetAngle;  // "no-reset" op-modes remember this targetAngle over auto->teleop transition

    // keeping track of which "slots" we have balls in.
    // slot 0 is the "forward" slot when we started (angle = 0)
    // slot 1 is the next one after that if we spindex "backwards" / non-shoot direction (so 1 position CCW from slot 0)
    // slot 2 is the next slot CCW from slot 1 (aka the slot 120degrees CW from slot 0)
    public enum SlotContent {Nothing, Purple, Green};
    SlotContent[] slots;  // this always has 3 elements: 0, 1 and 2

    double lastHue;
    LinkedList<Boolean> recentColors;
    LinkedList<Boolean> recentDist;

    public static double TOLERENCE_DEG_SHOOT = 2.0;
    public static double TOLERENCE_DEG_INDEX = 10.0;
    public static double MANUAL_DIVISOR = 10;
    public static double STEP_DEG = 120;
    // tuned December 10 with latest hardware rev (target collar, ramps, etc)
    public static PIDCoefficients shootPid = new PIDCoefficients(0.006, 0.02, 0.0003);
    public static PIDCoefficients storePid = new PIDCoefficients(0.005, 0.00, 0.0003);

    public Spindexer (HardwareMap hardwareMap) {
        spindexerMotor = new MotorEx(hardwareMap, "spindexer", Motor.GoBILDA.RPM_312);

        storeControl = new PIDController(storePid.p, storePid.i, storePid.d);
        shootControl = new PIDController(shootPid.p, storePid.i, storePid.d);
        storeControl.setTolerance(TOLERENCE_DEG_INDEX);
        shootControl.setTolerance(TOLERENCE_DEG_SHOOT);

        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stuckTime = new Timing.Timer(600, TimeUnit.MILLISECONDS);
        artifact_color = hardwareMap.digitalChannel.get("artifact_color");
        artifact_distance = hardwareMap.digitalChannel.get("artifact_distance");

        recentColors = new LinkedList<Boolean>();
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);

        recentDist = new LinkedList<Boolean>();
        recentDist.add(false);
        recentDist.add(false);
        recentDist.add(false);
        recentDist.add(false);

        // we always have 3 slots in this array
        slots = new SlotContent[]{
                SlotContent.Nothing,
                SlotContent.Nothing,
                SlotContent.Nothing
        };

        // the second brushland labs sensor, in analog mode
        analog = hardwareMap.analogInput.get("artifact_hsv");
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

    public boolean recentPurple() {
        boolean x = false;
        for (boolean rc : recentColors) {
            x |= rc;
        }
        return x;
    }

    public void clearRecentDist() {
        recentDist.clear();
        recentDist.add(false);
        recentDist.add(false);
        recentDist.add(false);
        recentDist.add(false);
    }

    public boolean artifactInSlot() {
        return (slots[currentSlot()] != SlotContent.Nothing);
    }

    public boolean haveArtifact() {
        boolean x = true;
        for (boolean rc : recentDist) {
            x &= rc;
        }
        return x;
    }

    public void spinShoot(){
        // todo: we should use the Shooter's ability to detect shots
        // to tell us when a shot went up .. meantime, we'll be
        // optimistic that anything in the "shoot" slot right now will
        // get shot...
        int shootIndex = currentSlot() + 1;
        if (shootIndex > 2) shootIndex = 0;
        if (slots[shootIndex] != SlotContent.Nothing) {
            // we probably shot
            slots[shootIndex] = SlotContent.Nothing;
        }

        spin = SpinDirection.Shoot;
        shootControl.reset();
        targetAngle += STEP_DEG;
        boostF = true;
        stuckTime.start();
    }

    public void spinIndex(){
        spin = SpinDirection.Index;
        storeControl.reset();
        targetAngle -= STEP_DEG;
        clearRecentDist();
    }

    // returns the index of the slot that's at the front of the robot;
    // at start this is slot 0
    public int currentSlot() {
        // we go from "targetAngle" because these are always whole
        // numbers .. so it'll be "a lie" until we're "at" a slot
        int norm = targetAngle;
        // java disagrees with others on what negatuve numbers do here.
        // for example, -120 % 360 = 240
        // (which is the same as spinning positive twice) but Java says that's -240
        norm = norm % 360;
        if (norm < 0) norm += 360; // account for weird java behavior
        if (norm == 0) return 0;
        if (norm == 240) return 1;
        if (norm == 120) return 2;
        return 0;
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

    public boolean isFull() {
        return !hasOpenSlot();
    }

    public boolean isEmpty() {
        for (SlotContent s : slots) {
            if (s != SlotContent.Nothing)
                return false;
        }
        return true;
    }

    public boolean hasOpenSlot() {
        for (SlotContent s : slots) {
            if (s == SlotContent.Nothing)
                return true;
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
        lastHue = (analog.getVoltage() / 3.3) * 360.0;
        recentColors.addLast((lastHue >= 150.0 && lastHue <= 185.0));
        //recentColors.addLast(artifact_color.getState());
        recentColors.removeFirst();
        recentDist.addLast(artifact_distance.getState());
        recentDist.removeFirst();
    }

    @Override
    public void periodic() {
        if (mode == Mode.Manual){
            spindexerPower = manualPower;
        }
        else{
            // for CCW ("shoot") direction, we use a separate PID .. so we
            // ned to know "which direction" we're spinning.
            PIDController control = (spin == SpinDirection.Index ? storeControl : shootControl);
            spindexerPower = control.calculate(currentAngle - targetAngle);

            // note: it's important to call .calculate() on our controller
            // _before_ we ask "atTarget()" so we have current information
            // from _this_ loop

            // let spindexer decide if there's something at the current
            // slot (but only if we also believe we are actually AT the
            // current slot)
            if (atTarget()) {
                if (haveArtifact()) {
                    if (slots[currentSlot()] == SlotContent.Nothing) {
                        slots[currentSlot()] = recentPurple() ? SlotContent.Purple : SlotContent.Green;
                    }
                }
            }

            storeControl.setPID(storePid.p, storePid.i, storePid.d);
            shootControl.setPID(shootPid.p, shootPid.i, shootPid.d);

            //if (spindexerPower > 0.5) spindexerPower = 0.5;
            //if (spindexerPower < -0.5) spindexerPower = -0.5;

            // temporary "boost" for the shoot-direction .. if we've "not
            // yet passed our goal" AND boostF is still true, we add extra
            // power (because the launched needs to have more power right
            // when it's super close to its goal).
            if (boostF && spin == SpinDirection.Shoot) {
                if (currentAngle < targetAngle) {//(spindexerPower > 0.0) {
                    spindexerPower += boostAmount;
                } else {
                    // we've passed our setpoint (at least once) because
                    // power went negative
                    boostF = false;
                }
            }

        }

        spindexerMotor.set(spindexerPower);
    }



    // leave this here for easy copy-pasting when creating a new command
    public class CommandTemplate extends CommandBase {
        //public void initialize() {}
        //public void execute() {}
        //public boolean isFinished() { return false; }
        //public void end(boolean interrupted){}
    }



    // spins in the Index direction once, and waits for completion
    public class IndexOnce extends CommandBase {
        public IndexOnce() {
            addRequirements(Spindexer.this);
        }
        public void initialize() {
            spinIndex();
        }
        public boolean isFinished() {
            return atTarget();
        }
    }


    public class ShootOnce extends CommandBase {
        public ShootOnce() {
            addRequirements(Spindexer.this);
        }
        public void initialize() {
            spinShoot();
        }
        public boolean isFinished() {
            return atTarget();
        }
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
        telem.log("spindexer-current-slot", currentSlot());
        telem.log("spindexer-at-target", atTarget());
        telem.log("spindexer-power", spindexerPower);
        telem.log("spindexer-stuck", isStuck());
        telem.log("spindexer-purple", recentColors.getFirst());
        telem.log("spindexer-analog", lastHue);
        telem.log("spindexer-have-artifact-debug", recentDist);
        telem.log("spindexer-have-artifact", haveArtifact());
        telem.log("spindexer-slot-0", slots[0]);
        telem.log("spindexer-slot-1", slots[1]);
        telem.log("spindexer-slot-2", slots[2]);
        telem.logDrivers("SPINDEX",renderSlot(0) + renderSlot(1) + renderSlot(2));
    }

    //temp
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
            if (operator.wasJustPressed(GamepadKeys.Button.A)) {
                spinShoot();
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                spinIndex();
            }
            manualPower = operator.getLeftX()/ MANUAL_DIVISOR;
            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                if (mode == Mode.Auto){
                    mode = Mode.Manual;
                }
                else{
                    mode = Mode.Auto;
                    reset();
                }
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                slots [0] = SlotContent.Nothing;
                slots [1] = SlotContent.Nothing;
                slots [2] = SlotContent.Nothing;
            }
/*
kind of for high-speed shoot debugging
            if (operator.wasJustPressed(GamepadKeys.Button.X)) {
                spin = SpinDirection.Shoot;
                shootControl.reset();
                targetAngle += (3 * STEP_DEG);
                stuckTime.start();
            }
*/
        }
    }
}
