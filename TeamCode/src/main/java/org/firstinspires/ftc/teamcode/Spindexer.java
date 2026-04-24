package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public static double BOOST_AMOUNT = 0.25;
    public static double PIN_ANGLE = -50.0;
    public static double SHORT_ANGLE = 25.0;
    public static double PIN_WAIT = 0.250;
    public static double SPIT_WAIT = 0.750;
    // if we want this lower, have to re-tune the PIDs (jan 22)
    public static double TOLERENCE_DEG = 10.0;
    public static double MANUAL_DIVISOR = 10;
    public static double STEP_DEG = 120;
    public static PIDCoefficients pid = new PIDCoefficients(0.0055, 0.0, 0.0003);
    public static double pid_f = 0.025; //0.026; // tuned at 0.03 but that twitched a little

    Servo indicatorLight;
    //public MotorEx spindexerMotor;
    public FloodMotor spindexerMotor;
    RevColorSensorV3 colorBack = null;
    RevColorSensorV3 colorFront = null;

    // colors for indicator light
    double RED = 0.28;
    double YELLOW = 0.35;
    double GREEN = 0.5;
    double PINK = 0.71;

    // stuff we read from sensors
    double currentAngle;    // computed from our encoder
    AnalogInput frontBeamBreak;
    AnalogInput backBeamBreak;
    AnalogInput intakeBeamBreak;
    
    // for beam breaks
    double lastFrontVoltage;
    double lastBackVoltage;
    double lastIntakeVoltage;
    boolean prevIntake = false;
    boolean thisIntake = false;
    
    // track beam-break status over several timesteps
    LinkedList<Boolean> recentFront;
    LinkedList<Boolean> recentBack;
    
    // color sensors
    float[] hsvBack = new float[3];
    float[] hsvFront = new float[3];
    boolean pinBalls = false;
    boolean shortSpindex = false;

    // stuff we derive
    public enum IntakeState {Waiting, BallEntering}

    public enum SpinDirection {Shoot, Index}
    SpinDirection spin = SpinDirection.Index;

    public enum Mode {Auto, Manual}
    Mode mode = Mode.Auto;
    double manualPower;

    Timing.Timer stuckTime;

    public PIDController control;
    public boolean boostF = false;
    private boolean _atTarget = false; // were we atTarget() last loop, after .calculate()?

    double spindexerPower;
    public int targetAngle;  // "no-reset" op-modes remember this targetAngle over auto->teleop transition

    // keeping track of which "slots" we have balls in.
    // slot 0 is the "forward" slot when we started (angle = 0)
    // slot 1 is the next one after that if we spindex "backwards" / non-shoot direction (so 1 position CCW from slot 0)
    // slot 2 is the next slot CCW from slot 1 (aka the slot 120degrees CW from slot 0)
    public enum SlotContent {Nothing, Unknown, Purple, Green};
    public SlotContent[] slots;  // this always has 3 elements: 0, 1 and 2

    // use front beam-break to (attempt to) count balls in
    int ballCounter = 0;
    private IntakeState intakeState;

    public Spindexer (HardwareMap hardwareMap) {
	// note: using the "FloodMotor" here since that caps "max
	// change in power" -- to overcome the Floodgate switch
	// hardware issues originally, but right now to stop the
	// spindexer doing crazy stuff
        spindexerMotor = new FloodMotor(hardwareMap, "spindexer");//, Motor.GoBILDA.RPM_312);
        indicatorLight = hardwareMap.get(Servo.class, "indicator");

        control = new PIDController(pid.p, pid.i, pid.d);
        control.setTolerance(TOLERENCE_DEG);

        spindexerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        stuckTime = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

        // we always have 3 slots in this array
        slots = new SlotContent[]{
                SlotContent.Nothing,
                SlotContent.Nothing,
                SlotContent.Nothing
        };

        // always have 5 items in these
        recentFront = new LinkedList<Boolean>();
        recentBack = new LinkedList<Boolean>();
        resetBeamBreaks();

        // the two brushland labs sensors (i2c mode because not enough analog ports) 
        colorBack = hardwareMap.get(RevColorSensorV3.class, "color_back");
        colorFront = hardwareMap.get(RevColorSensorV3.class, "color_front");

        //beam break sensors
        frontBeamBreak = hardwareMap.analogInput.get("front_beam_break");
        backBeamBreak = hardwareMap.analogInput.get("back_beam_break");
        intakeBeamBreak = hardwareMap.analogInput.get("intake_beam_break");

        //for counting balls via intake beambreak
        intakeState = IntakeState.Waiting;
        reset();
    }

    public void reset() {
        currentAngle = 0;
        targetAngle = 0;
        spindexerMotor.set(0);
	control.setSetPoint(0);
        // drive-team needs to orient Spindexer with one segment forward
        //spindexerMotor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //spindexerMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.resetEncoder();
    }

    //private static double TICKS_PER_REV = 384.5;  // 435
    private static double TICKS_PER_REV = 537.7;  // 312

    private double ticksToDeg(double ticks){
	double motorRevs = ticks / TICKS_PER_REV;
	return motorRevs * 360.0;
    }

    public boolean artifactInSlot() {
        return (slots[currentSlot()] != SlotContent.Nothing);
    }

    public void assumeFrontArtifact() {
        slots[currentSlot()] = SlotContent.Unknown;
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

    public void resetBeamBreaks() {
        recentFront.clear();
        recentFront.add(false);
        recentFront.add(false);
        recentFront.add(false);

        recentBack.clear();
        recentBack.add(false);
        recentBack.add(false);
        recentBack.add(false);
    }

    public boolean haveArtifactFront() {
        for (boolean b : recentFront) {
            if (!b) return false;
        }
        return true;
    }
    public boolean haveArtifactBack() {
	for (boolean b : recentBack) {
            if (!b) return false;
        }
        return true;
    }
    public boolean haveFrontAndBack(){
        if (haveArtifactBack() && haveArtifactFront()){
            return true;
        }
        return false;
    }

    public void spinShoot(){
        pinBalls = false;
	shortSpindex = false;
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
	control.setSetPoint(targetAngle);
        boostF = true;
        stuckTime.start();
    }

    public void spinIndex(){
        spin = SpinDirection.Index;
        pinBalls = false;
        control.reset();
        targetAngle -= STEP_DEG;
	control.setSetPoint(targetAngle);
    }

    // rotate "less far" when we're still intaking
    public void shortSpin() {
	shortSpindex = true;
    }

    // "pin" the balls against the finger when we're full
    public void pinBalls(){
        pinBalls = true;
	shortSpindex = false;
    }

    public void unPinBalls(){
        pinBalls = false;
    }

    public void spinModeIndex() {
        spin = SpinDirection.Index;
        pinBalls = false;
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
            if (diff > TOLERENCE_DEG){
                return true;
            }
        }
        return false;
    }

    public void unStick() {
	// go to the 'nearest' 120 increment
	// (could / should we use 'spin' mode for which way?)
	if (targetAngle > currentAngle) {
	    // bring target angle LESS than current angle
	    int where = (int)currentAngle;
	    int extra = where % 120;
	    targetAngle = where - extra;
	    spinModeIndex();
	} else {
	    // make targetAngle MORE than current angle
	    int where = (int)currentAngle;
	    int extra = where % 120;
	    targetAngle = where + (120 - extra);
	}
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

    public void updateIntakeState(){
        switch(intakeState) {
            case Waiting:
                if (thisIntake) {
                    ballCounter++;
                    intakeState = IntakeState.BallEntering;
                }
                break;

            case BallEntering:
                if (!thisIntake){
                    intakeState = IntakeState.Waiting;
                }
                break;
        }
    }

    // reading from I2C devices is slow, so we only do this sometimes
    public void readSlotColors() {
        if (slots[currentSlot()] == SlotContent.Unknown) {
            Color.RGBToHSV(colorFront.red(), colorFront.green(), colorFront.blue(), hsvFront);
            // check color, change slots[currentSlot() ]
        }
        if (slots[currentBackSlot()] == SlotContent.Unknown) {
            Color.RGBToHSV(colorBack.red(), colorBack.green(), colorBack.blue(), hsvBack);
            // check array, set it
        }
    }

    public void read_sensors(double time) {
        currentAngle = ticksToDeg(spindexerMotor.getCurrentPosition());
        lastFrontVoltage = (frontBeamBreak.getVoltage());
        lastBackVoltage = (backBeamBreak.getVoltage());
        lastIntakeVoltage = (intakeBeamBreak.getVoltage());

        prevIntake = thisIntake;
        thisIntake = (lastIntakeVoltage < 1.0);

	if (_atTarget) {
	    recentFront.addLast(lastFrontVoltage < 1.0);
	    recentFront.removeFirst();
	    recentBack.addLast(lastBackVoltage < 1.0);
	    recentBack.removeFirst();
	}
    }


    @Override
    public void periodic() {
        // have to set these each loop in case we're setting from Panels/Dashboard
        control.setTolerance(TOLERENCE_DEG);
	control.setPID(pid.p, pid.i, pid.d);

	// note: positive power to spindexerPower is "shoot" direection

        if (mode == Mode.Manual){
            spindexerPower = manualPower;
        } else {
	    // we "pin" the ball against the launch finger if we're
	    // full, so add in some angle adjustment
            double moreAngle = 0.0;
	    if (pinBalls) {
		moreAngle = PIN_ANGLE;
	    } else if (shortSpindex) {
		moreAngle = SHORT_ANGLE;
	    }
            spindexerPower = control.calculate(currentAngle + moreAngle);
            spindexerPower += (pid_f * Math.signum(spindexerPower));

            // note: it's important to call .calculate() on our controller
            // _before_ we ask "atTarget()" so we have current information
            // from _this_ loop
	    boolean lastAtTarget = _atTarget;
	    _atTarget = atTarget();
	    if (!lastAtTarget && _atTarget) {
		// we _just_ arrived at our target .. reset the
		// beam-break arrays so we need all cycles of 'real'
		// beak-breaking before we consider a ball there
		// (since they'll be broken the whole time we're
		// rotating, mostly)
		resetBeamBreaks();
	    }

            // when the spindexer thinks it's settled, we look at BOTH beambrakes and fill those two slots
            // if they're broken.
            // we look at both so that a ball moving over them doesn't cause a miss-count
            if (spin == SpinDirection.Index && _atTarget) {
                if (haveFrontAndBack()) {
                    slots[currentSlot()] = SlotContent.Unknown;
                    slots[currentBackSlot()] = SlotContent.Unknown;
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

            // temporary "boost" for the shoot-direction .. if we've "not
            // yet passed our goal" AND boostF is still true, we add extra
            // power (because the launched needs to have more power right
            // when it's super close to its goal).

	    // note: shoot direction is positive angles, and positive power
            if (boostF && spin == SpinDirection.Shoot) {
                if (currentAngle < targetAngle) {
                    spindexerPower += BOOST_AMOUNT;
                } else {
                    // we've passed our setpoint (at least once)
                    boostF = false;
                }
            }
        }

	/*
        if (spindexerPower < 0.0 && spin == SpinDirection.Shoot) {
            spindexerPower = 0.0;
        }
	*/

	if (false && isStuck()) {
	    spindexerPower = 0.0;
	}

        spindexerMotor.set(spindexerPower);
        updateIntakeState();
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
            return _atTarget;
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
            return _atTarget;
        }
    }

    private String renderSlot(int i) {
        String s = "[ ";
        if (slots[i] == SlotContent.Nothing) s += "     ]";
        if (slots[i] == SlotContent.Purple) s +=  "PPPP ]";
        if (slots[i] == SlotContent.Green) s += "GGGG ]";
        if (slots[i] == SlotContent.Unknown) s += "**** ]";
        return s;
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("spindexer-pin", pinBalls);
        telem.log("spindexer-ticks", spindexerMotor.getCurrentPosition());
        telem.log("spindexer-target-angle", targetAngle);
        telem.log("spindexer-current-angle", currentAngle);
        telem.log("spindexer-current-slot", currentSlot());
        telem.log("spindexer-at-target", _atTarget);
        telem.log("spindexer-power", spindexerPower);
        telem.log("spindexer-stuck", isStuck());
        telem.log("spindexer-have-artifact", haveFrontAndBack());
        telem.log("spindexer-slot-0", slots[0]);
        telem.log("spindexer-slot-1", slots[1]);
        telem.log("spindexer-slot-2", slots[2]);
        telem.log("spindexer-spin", spin);
        //telem.logBoth("spindexer-beam-intake", lastIntakeVoltage);
        telem.logBoth("spindexer-beam-front", lastFrontVoltage);
        telem.logBoth("spindexer-beam-back", lastBackVoltage);
        //telem.logBoth("spindexer-ballcount", ballCounter);
        telem.logBoth("spindexer-artifacts", artifactCount());
	telem.logBoth("spindexer-recent-front", recentFront);
	telem.logBoth("spindexer-recent-back", recentBack);
        telem.log("spindexer-color-back", hsvBack[0]);
        telem.log("spindexer-color-front", hsvFront[0]);
        telem.logDrivers("SPINDEX",renderSlot(0) + renderSlot(1) + renderSlot(2));
    }

    public class ManualAdjust extends CommandBase {
        GamepadEx operator;
        
        public ManualAdjust(GamepadEx operator) {
            this.operator = operator;
            addRequirements(Spindexer.this);
        }
        @Override
        public void execute() {
            manualPower = -(operator.getLeftX() / MANUAL_DIVISOR);
            mode = Mode.Manual;
        }
        public void end(boolean inturupted){
            mode = Mode.Auto;
            reset();
        }
    }
    public CommandBase manualAdjust(GamepadEx operator) {
        return new Spindexer.ManualAdjust(operator);
    }

    public class ResetContents extends CommandBase {
        public ResetContents() {
            addRequirements(Spindexer.this);
        }
        
        @Override
        public void execute() {
            slots [0] = SlotContent.Nothing;
            slots [1] = SlotContent.Nothing;
            slots [2] = SlotContent.Nothing;
        }
    }
    public CommandBase resetContents() {
        return new ResetContents();
    }
}
