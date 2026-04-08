package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.LinkedList;
import java.util.concurrent.TimeUnit;

@Configurable
public class Spindexer extends SubsystemBase {
    public MotorEx spindexerMotor;
    Servo indicatorLight;

    // colors for indicator light
    double RED = 0.28;
    double YELLOW = 0.35;
    double GREEN = 0.5;
    double PINK = 0.71;

    // stuff we read from sensors
    double currentAngle;    // computed from our encoder
    AnalogInput analog_hsv;
    AnalogInput analog_distance;
    AnalogInput frontBeamBreak;
    AnalogInput backBeamBreak;
    AnalogInput intakeBeamBreak;
    // for beam breaks
    double lastFrontVoltage;
    double lastBackVoltage;
    double lastIntakeVoltage;

    // stuff we derive
    public enum SpinDirection {Shoot, Index}
    SpinDirection spin = SpinDirection.Index;
    public enum Mode {Auto, Manual}
    Mode mode = Mode.Auto;
    double manualPower;
    Timing.Timer stuckTime;

    public PIDController control;
    public boolean boostF = false;
    public static double boostAmount = 0.5;//0.85;

    double spindexerPower;
    public int targetAngle;  // "no-reset" op-modes remember this targetAngle over auto->teleop transition

    // keeping track of which "slots" we have balls in.
    // slot 0 is the "forward" slot when we started (angle = 0)
    // slot 1 is the next one after that if we spindex "backwards" / non-shoot direction (so 1 position CCW from slot 0)
    // slot 2 is the next slot CCW from slot 1 (aka the slot 120degrees CW from slot 0)
    public enum SlotContent {Nothing, Purple, Green};
    public SlotContent[] slots;  // this always has 3 elements: 0, 1 and 2

    double lastHue;
    double lastDistance;
    LinkedList<Boolean> recentColors;
    LinkedList<Double> recentDist;

    // if we want this lower, have to re-tune the PIDs (jan 22)
    public static double TOLERENCE_DEG = 10.0;
    public static double MANUAL_DIVISOR = 10;
    public static double STEP_DEG = 120;
    public static double DISTANCE_THRESHOLD = 20.0;
    public static int DISTANCE_WINDOW = 3;
    // tuned December 10 with latest hardware rev (target collar, ramps, etc)
    //public static PIDCoefficients pid = new PIDCoefficients(0.006, 0.02, 0.0003);
    // more aggressive feb 2
    public static PIDCoefficients pid = new PIDCoefficients(0.008, 0.02, 0.0003);
    public static double pid_f = 0.026; // tuned at 0.03 but that twitched a little

    public Spindexer (HardwareMap hardwareMap) {
        spindexerMotor = new MotorEx(hardwareMap, "spindexer", Motor.GoBILDA.RPM_312);
        indicatorLight = hardwareMap.get(Servo.class, "indicator");

        control = new PIDController(pid.p, pid.i, pid.d);
        control.setTolerance(TOLERENCE_DEG);

        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stuckTime = new Timing.Timer(670, TimeUnit.MILLISECONDS);

        recentColors = new LinkedList<Boolean>();
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);
        recentColors.add(false);

        recentDist = new LinkedList<Double>();

        // we always have 3 slots in this array
        slots = new SlotContent[]{
                SlotContent.Nothing,
                SlotContent.Nothing,
                SlotContent.Nothing
        };

        // the two brushland labs sensors, in analog mode (now)
        analog_hsv = hardwareMap.analogInput.get("artifact_hsv");
       // analog_distance = hardwareMap.analogInput.get("artifact_distance");
        //beam break sensors
        frontBeamBreak = hardwareMap.analogInput.get("front_beam_break");
        backBeamBreak = hardwareMap.analogInput.get("back_beam_break");
        intakeBeamBreak = hardwareMap.analogInput.get("intake_beam_break");
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
         //   x |= rc;
            x = rc;
        }
        return x;
    }

    public void clearRecentDist() {
        while (recentDist.size() > DISTANCE_WINDOW) {
            recentDist.removeFirst();
        }
    }

    public boolean artifactInSlot() {
        return (slots[currentSlot()] != SlotContent.Nothing);
    }

    public int artifactCount() {
        int count = 0;
        for (SlotContent s : slots) {
            if (s != SlotContent.Nothing) {
                count += 1;
            }
        }
        return count;
    }

    public boolean haveArtifactFront() {
        if (lastFrontVoltage > 1.0){
            return true;
        }
        return false;
    }
    public boolean haveArtifactBack() {
        if (lastBackVoltage > 1.0){
            return true;
        }
        return false;
    }
    public boolean haveArtifactIntake() {
        if (lastIntakeVoltage > 1.0){
            return true;
        }
        return false;
    }


    public void spinShoot(){
        // todo: we should use the Shooter's ability to detect shots
        // to tell us when a shot went up .. meantime, we'll be
        // optimistic that anything in the "shoot" slot right now will
        // get shot...
        int shootIndex = currentShootSlot();
        if (slots[shootIndex] != SlotContent.Nothing) {
            // we probably shot
            slots[shootIndex] = SlotContent.Nothing;
        }

        spin = SpinDirection.Shoot;
        control.reset();
        targetAngle += STEP_DEG;
        boostF = true;
        stuckTime.start();
    }

    public void spinIndex(){
        spin = SpinDirection.Index;
        control.reset();
        targetAngle -= STEP_DEG;
        recentDist.clear();
    }

    public void spinModeIndex() {
        spin = SpinDirection.Index;
        control.reset();
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
    public int currentBackSlot(){
        int slot = currentSlot() + 1;
        if (slot > 2){
            slot = 0;
        }
        return slot;
    }
    public boolean haveFrontAndBack(){
        if (haveArtifactBack() && haveArtifactFront()){
            return true;
        }
        return false;
    }

    // the "shoot slot" is the next one after the currentSlot() but
    // wrapping..
    public int currentShootSlot() {
        int shootIndex = currentSlot() + 1;
        if (shootIndex > 2) shootIndex = 0;
        return shootIndex;
    }

    // if we have at least one green ball
    public boolean haveOneGreen() {
        for (SlotContent sc : slots) {
            if (sc == SlotContent.Green) {
                return true;
            }
        }
        return false;
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

    public int countArtifacts(){
        int count = 0;
        for (SlotContent s : slots){
            if (s != SlotContent.Nothing){
                count += 1;
            }
        }
        return count;
    }

    public int firstFullSlot(){
        int index = 0;
        for (SlotContent s : slots){
            if (s != SlotContent.Nothing){
                return index;
            }
            index += 1;
        }
        return -1;
    }
    public int firstEmptySlot(){
        int index = 0;
        for (SlotContent s : slots){
            if (s == SlotContent.Nothing){
                return index;
            }
            index += 1;
        }
        return -1;
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

    public boolean atTarget() {
        if (spin == SpinDirection.Shoot) {
            return control.atSetPoint() || control.getPositionError() < 0.0;
        }
        return control.atSetPoint();
    }

    public void read_sensors(double time) {
        currentAngle = ticksToDeg(spindexerMotor.getCurrentPosition());
        lastHue = (analog_hsv.getVoltage() / 3.3) * 360.0;
        recentColors.addLast((lastHue >= 130.0 && lastHue <= 185.0));
        //recentColors.addLast(artifact_color.getState());
        recentColors.removeFirst();
        lastDistance = (analog_distance.getVoltage() / 3.3) * 100.0;
        recentDist.addLast(lastDistance);
        lastFrontVoltage = (frontBeamBreak.getVoltage());
        lastBackVoltage = (backBeamBreak.getVoltage());
        lastIntakeVoltage = (intakeBeamBreak.getVoltage());
        clearRecentDist();
    }

/*

bug from adrian

feathering "intake" mode
if interrupt "during" spin then it gets confused about which slot is what

*/

    @Override
    public void periodic() {
        // have to set these each loop in case we're setting from Panels/Dashboard
        control.setTolerance(TOLERENCE_DEG);
        if (mode == Mode.Manual){
            spindexerPower = manualPower;
        }
        else{
            spindexerPower = control.calculate(currentAngle - targetAngle);
            spindexerPower += (pid_f * Math.signum(spindexerPower));

            // note: it's important to call .calculate() on our controller
            // _before_ we ask "atTarget()" so we have current information
            // from _this_ loop

            // let spindexer decide if there's something at the current
            // slot (but only if we also believe we are actually AT the
            // current slot)
            if (spin == SpinDirection.Index && atTarget()) {
                if (haveFrontAndBack()) {
                    slots[currentSlot()] = SlotContent.Purple; //Fix me: correct colour
                    slots[currentBackSlot()] = SlotContent.Purple; //"                "
                }
            }

            // indicator lights
            // kind-of traffic lights, by number of balls:
            // 0 - off
            // 1 - red
            // 2 - yellow
            // 3 - green
            // additionally, we may do something if we're "currently sorting"
            switch (artifactCount()) {
                case 0:
                    indicatorLight.setPosition(0.0);
                    break;
                case 1:
                    indicatorLight.setPosition(RED);
                    break;
                case 2:
                    indicatorLight.setPosition(YELLOW);
                    break;
                case 3:
                    indicatorLight.setPosition(GREEN);
                    break;
            }

            control.setPID(pid.p, pid.i, pid.d);

            //if (spindexerPower > 0.5) spindexerPower = 0.5;
            //if (spindexerPower < -0.5) spindexerPower = -0.5;

            // temporary "boost" for the shoot-direction .. if we've "not
            // yet passed our goal" AND boostF is still true, we add extra
            // power (because the launched needs to have more power right
            // when it's super close to its goal).

            if (slots[currentShootSlot()] == SlotContent.Nothing) {
                // if we have an empty slot coming up to shoot, we
                // _don't_ want to apply the boost
                boostF = false;
            }
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

        if (spindexerPower < 0.0 && spin == SpinDirection.Shoot) {
            spindexerPower = 0.0;
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
        if (slots[i] == SlotContent.Nothing) s += "     ]";
        if (slots[i] == SlotContent.Purple) s +=  "PPPP ]";
        if (slots[i] == SlotContent.Green) s += "GGGG ]";
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
        telem.log("spindexer-analog-hue", lastHue);
        telem.log("spindexer-analog-distance", lastDistance);
        telem.log("spindexer-have-artifact-debug", recentDist);
        telem.log("spindexer-have-artifact", haveFrontAndBack());
        telem.log("spindexer-slot-0", slots[0]);
        telem.log("spindexer-slot-1", slots[1]);
        telem.log("spindexer-slot-2", slots[2]);
        telem.log("spindexer-spin", spin);
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
           /* if (operator.wasJustPressed(GamepadKeys.Button.A)) {
                spinShoot();
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                spinIndex();
            }*/
            manualPower = operator.getLeftX()/ MANUAL_DIVISOR;
            if (operator.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                    mode = Mode.Manual;
            }
            else{
                if (mode == Mode.Manual){
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
                control.reset();
                targetAngle += (3 * STEP_DEG);
                stuckTime.start();
            }
*/
        }
    }
}
