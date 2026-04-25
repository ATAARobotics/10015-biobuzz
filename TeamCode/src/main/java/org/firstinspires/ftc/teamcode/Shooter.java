package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.LinkedList;

@Configurable
public class Shooter extends SubsystemBase {
    MotorEx motor0;
    MotorEx motor1;
    MotorGroup shooterMotor;
    Servo hood;
    double ticks;
    double ticksPerSecond;
    double power;
    double appliedVoltage; // proportion of batteries current voltage needed to achieve rpm target (based on flywheel testing)
    double tbhOutput = 0.0;
    double tbhLastCrossedOutput = 0.0;
    double tbhLastError = 0.0;
    public static double TBH_GAIN = 0.000007;

    // shot-counter
    int shotsFired = 0;

    class RpmData {
        public double time;
        public double rpm;
        public RpmData(double t, double r) {
            time = t;
            rpm = r;
        }
    }
    LinkedList<RpmData> recentRpms;
    public static double RPM_WINDOW_LENGTH = 0.150;  // seconds
    // the algorithm we use is:
    // - a bucket of recent samples, at most 200ms in length
    // - if the oldest rpm minus the newest rpm shows a >400 rpm drop, that's a shot
    // looking at telemetry data, we determined that this only happens
    // during shots (normal spin-down is slower so the 200ms window can't see it)

    double hoodSlope;
    public static double RPM_LOW = 2600;
    public static double RPM_HIGH = 3800;
    public static double RPM_PERCENT = 1;
    public static double HOOD_MAX = 0.9;
    public static double HOOD_MIN = 0.2;
    public static double HOOD_MAX_DEGREES = 45.0;
    public static double HOOD_MIN_DEGREES = 30.0;
    public static double MANUAL_RPM = 0;
    public static double MANUAL_HOOD = 0.00;
    public static double FAR_RPM = 1111; // FIXME TODO temp for testing
    public static double FAR_HOOD = 0.50;
    //public static double RPM_VS_DIST_SLOPE = 39.8; // old is 20.086
    //public static double RPM_VS_DIST_INTERCEPT = 1022; //Old is 2411.7
    //public static double RPM_VS_DIST_SLOPE = 37.8; // down 10%
    //public static double RPM_VS_DIST_INTERCEPT = 1124; //up 10%
    public double rpmSlope;
    public double rpmIntercept;
    public static int RPM_DROP_FOR_SHOT = 200;  // how many RPMs must drop for "a shot" to be counted
    public static double FAR_DISTANCE = 125.0;

    public static double BAND = 500;
    public static double BANG_POWER = 1.0;
    public static double RPM_TOLERANCE = 50;
    public static double RPM_TOLERANCE_OVER = 250;
    public static double POWER_OVERRIDE = 0.0;
    private static final double TICKS_PER_REV = 28.0;  // fixme: get from motor

    // tuned april 24, fresh battery and replaced shooter bearings
    public static double P = 0.001;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F_LOW = 0.72; // at 2600rpm
    public static double F_HI = 1.0; // at 3800rpm
    public PIDController control;
    
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double targetRpm;
    double targetHoodAngle = 30.75;
    double targetHood = HOOD_MIN;
    double currentRpm;
    double voltage; // current battery voltage
    public boolean autoRpm = false;
    public double aprilDistance;
    //public double geometricDistance;

    //public static double kv = 0.002213; //kv is Feed Forward Model slope, determined experimentally with flywheel
    //public static double ks = 0.129514; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)

    // tuned kv and ks on April 20
    public static double kv = 0.0023;
    public static double ks = 2.3245;
    MovingAverage rpmFilter;
    public static int RPM_FILTER_SIZE = 8;
    double smoothRpm;


    public Shooter(HardwareMap hardwareMap) {
        // do any one-time initialization here

        motor0 = new MotorEx(hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor0.setInverted(false);

        motor1 = new MotorEx(hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor1.setInverted(true);

        shooterMotor = new MotorGroup(motor0, motor1);
        targetRpm = 0;
        battery = hardwareMap.voltageSensor.get("Control Hub");  // FIXME: move to OpMode?

	control = new PIDController(P, I, D);

        hood = hardwareMap.get(Servo.class, "hood");
        recentRpms = new LinkedList<RpmData>();
	rpmFilter = new MovingAverage(RPM_FILTER_SIZE);
    }

    public void reset() {
        stop();
    }

    public void stop() {
        targetRpm = 0;
        motor0.set(0);
        motor1.set(0);
    }
    public void read_sensors(double time) {
        // get any inputs from our encoders or other sensors
        // ("shooterL" is in port 2, a hardware port, "shooterR" is in port 3)
        ticks = motor1.getCurrentPosition();
        ticksPerSecond = motor1.getVelocity();
        currentRpm = (ticksPerSecond * 60) / TICKS_PER_REV;
        voltage = battery.getVoltage();

        // recent RPM data for shot-counter.
        recentRpms.addLast(new RpmData(time, currentRpm));
        smoothRpm = rpmFilter.add(currentRpm);
        // ensure we only have 200ms or less worth of data
        while (time - recentRpms.getFirst().time > RPM_WINDOW_LENGTH) {
            recentRpms.removeFirst();
        }
    }


    // leave this here for easy copy-pasting when creating a new command
    public class CommandTemplate extends CommandBase {
        //public void initialize() {}
        //public void execute() {}
        //public boolean isFinished() { return false; }
        //public void end(boolean interrupted){}
    }
    public double rpmMin(){
        return 41.9 * aprilDistance + 351;
    }
    public double rpmMax(){
        rpmSlope = (RPM_HIGH - RPM_LOW) / (140 - 48.5);
        rpmIntercept = RPM_LOW + (rpmSlope * -48.5);
        return (rpmSlope * aprilDistance) + rpmIntercept;
    }
    public double hoodAngle(double rpm) {
        // linear fit of data from april 17
        return 0.01 * rpm + 8.15;
        //tuned feb 16
      //  return -9e8 * (rpm * rpm) + 0.001 * rpm - 2.03;
    }

    public double hoodAngleLinear(double rpm) {
	double percent =  rpm / (RPM_HIGH - RPM_LOW);
	double range = (HOOD_MAX_DEGREES - HOOD_MIN_DEGREES);
	double hood = HOOD_MIN + (percent * range);
	return hood;
    }

    public double degreeToServo(double degrees){
        // 0.20 == 30.75 degrees
        // 0.85 == 50.75 degrees
	double percent = (degrees - 30.75) / (50.75 - 30.75);
	double servo = ((0.85 - 0.20) * percent) + 0.20;
        if (servo < 0.20) servo = 0.20;
        if (servo > 0.85) servo = 0.85;
        return servo;
    }

    public boolean readyToShoot() {
        // diff will be positive number if we're below target, and
        // negative number if we're above target
        double rpmDiff = targetRpm - currentRpm;
        if (rpmDiff > 0) return rpmDiff < RPM_TOLERANCE;
        return -rpmDiff < RPM_TOLERANCE_OVER;
     ///   return (targetRpm > 0 && Math.abs(rpmDiff) < RPM_TOLERANCE);

 // linear equation for the minimun rpm to hit the target
//        return currentRpm > rpmMin() && targetRpm > 0;


        // if the shooter is still over-shooting or getting "stuck",
        // try a different "over" vs "under" tolerance
        /*
        if (rpmDiff > 0 && rpmDiff < RPM_TOLERANCE) {
            return true;
        }
        if (rpmDiff < 0 && -rpmDiff < RPM_TOLERANCE_OVER) {
            return true;
        }
        return false;
        */
    }

    public int getCurrentShots() {
        return shotsFired;
    }

    public void autoShootRpm() {
        autoRpm = true;
    }

    public void manualShootRpm() {
        autoRpm = false;
        targetRpm = 0;
    }

    @Override
    public void periodic() {
	control.setPID(P, I, D);
        // auto-computed RPM, optional
        if (autoRpm /*&& aprilDistance > 0.5*/) {

            // we have regression lines for "max" and "min" RPMs that
            // gets artifacts scored, and RPM_PERCENT controls where
            // we are between them (0% means min, 100% means max).
            targetRpm = rpmMax(); //went back to just rpm max april 17
          //  double dif = rpmMax() - rpmMin();
          //  targetRpm = rpmMin() + (RPM_PERCENT * dif);

            // "new idea" to target hood angle based of RPM, not distance
            //targetHood = hoodAngle(currentRpm);

            //targetHood = HOOD_COEF *Math.pow(aprilDistance, HOOD_EXP);
            //targetHood = -0.00006 * aprilDistance * aprilDistance + 0.0167 * aprilDistance - 0.4328;
        }
      /*  if (aprilDistance < 76.7){
            hoodSlope = 0.0004 + -0.0000205 * (aprilDistance - 76.7);  //hoodSlope = hoodSlope@76.7 + hoodSlopeSlope * (aprilDistance - a distance)
            targetHood = 0.4 + hoodSlope * (currentRpm - 3500);
        }
        else if (aprilDistance > 84){
            hoodSlope = 0.00083 + -0.000022 * (aprilDistance - 84);
            targetHood = 0.7 + hoodSlope * (currentRpm-4300);
        }
        else {
            hoodSlope = 0.0004 + 0.000059 * (aprilDistance - 76.7); // for 76.7 < aprilDistance < 84
            targetHood = 0.45 + hoodSlope * (currentRpm-4100);
        }*/

        // TODO we are special-casing the far-zone for now and not using the regression algorithm
        if (autoRpm && aprilDistance > FAR_DISTANCE) {
            //targetHood = FAR_HOOD;
            targetRpm = FAR_RPM;
        }
        if (targetRpm > 5300){
            targetRpm = 5300;
        }

        if (MANUAL_RPM > 1.0 ){//&& targetRpm > 0.0) {
            targetRpm = MANUAL_RPM;
        }
        if (MANUAL_HOOD > HOOD_MIN) {
            targetHood = MANUAL_HOOD;
        }

	// "take back half" computations
      /*  if (targetRpm == 0){
            tbhOutput = 0;
            tbhLastCrossedOutput = 0;
            tbhLastError = 0;
            power = 0;
        }
        else{
            if (tbhLastError == 0.0){
                appliedVoltage = (kv * targetRpm) + ks;
                tbhOutput = appliedVoltage/voltage;
                tbhLastCrossedOutput = tbhOutput;
            }
        }
        //tbh controller
        double error = targetRpm - currentRpm;
        tbhOutput += TBH_GAIN * error;
        tbhOutput = Math.max(0, Math.min(1,tbhOutput)); //Maintain a value between 0 and 1
        if (Math.signum(error) != Math.signum(tbhLastError) && tbhLastError != 0.0){
            tbhOutput = 0.5 * (tbhOutput + tbhLastCrossedOutput);
            tbhLastCrossedOutput = tbhOutput;
        }
        tbhLastError = error;
        power = tbhOutput;*/
      //  power = appliedVoltage/voltage;

	if (targetRpm > 0) {
	    control.setSetPoint(targetRpm);
	    power = control.calculate(currentRpm);
	    if (power > 0.00000001) {
		// we find that varying F from 0.72 up to 0.9
		// depending on the TARGET RPM seems to work well
		// .. so we want f to be "0.72" at 2500 RPM and "0.9" at
		// 3500 RPM.
		double percent = (targetRpm - RPM_LOW) / (RPM_HIGH - RPM_LOW);
		if (percent > 1.0) percent = 1.0;
		if (percent < 0.0) percent = 0.0;
		double f = F_LOW + ((F_HI - F_LOW) * percent);
		power += f;
	    }
	} else {
	    power = 0.0;
	}

     /*   if (currentRpm < (targetRpm - BAND)) {
            power = BANG_POWER;
        }*/

        // Above here can be messed with
        if (POWER_OVERRIDE > 0.0) {
            power = POWER_OVERRIDE;
        }

        if (power < 0) power = 0;
	targetHood = degreeToServo(hoodAngleLinear(smoothRpm));
	//targetHood = degreeToServo(targetHoodAngle);

        if (targetHood < HOOD_MIN){
            targetHood = HOOD_MIN;
        }
        if (targetHood > HOOD_MAX){
            targetHood = HOOD_MAX;
        }

        hood.setPosition(targetHood);
        shooterMotor.set(power);

        // count shots
        if (recentRpms.size() > 2) {
            double minRpm = 10000.0; // we can't spin this fast
            double maxRpm = 0.0;
            for (RpmData r : recentRpms) {
                if (r.rpm < minRpm) { minRpm = r.rpm; }
                if (r.rpm > maxRpm) { maxRpm = r.rpm; }
            }
            //double rpmDrop = maxRpm - minRpm;// recentRpms.getFirst().rpm - recentRpms.getLast().rpm;
            double rpmDrop = recentRpms.getFirst().rpm - recentRpms.getLast().rpm;
            if (rpmDrop > RPM_DROP_FOR_SHOT) {
                shotsFired += 1;
                recentRpms.clear();
            }
        }
    }

    public void addTelemetry(HyperTelemetry telem) {
        //telem.logBoth("motor0", power); //what you see on the screen
        //telem.logBoth("targetRpm", targetRpm);
        //telem.logBoth("Current RPM", currentRpm);
        //telem.logBoth("Applied Voltage", appliedVoltage);
        telem.log("shooter-shots-fired" , shotsFired);
        telem.log("shooter-rpm-target", targetRpm);
        telem.log("shooter-rpm-current", currentRpm);
        telem.log("shooter-power", power);
        telem.log("shooter-hood-degrees", hoodAngleLinear(smoothRpm));
        telem.log("shooter-hood-angle", degreeToServo(hoodAngleLinear(smoothRpm)));
        telem.log("shooter-auto-rpm", autoRpm);
        telem.log("shooter-distance", aprilDistance);
        telem.log("shooter-voltage", voltage);
        telem.log("shooter-ticks", ticks);
        telem.log("shooter-smooth-rpm", smoothRpm);
    }

    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        GamepadEx operator;

        public HumanInputs(GamepadEx operator, GamepadEx driver) {
            this.operator = operator;
            this.driver = driver;
            addRequirements(Shooter.this);
        }

        @Override
        public void execute() {
            // decide what to do based on sensors and human inputs from controller

            // FIXME: need operator controls ... and far-shot target?
            // and "hood" controls?

           /* if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                targetRpm = NEAR_RPM;
            }
            if (driver.wasJustPressed(GamepadKeys.Button.A)){
                targetRpm = 0;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                autoRpm = false;
                if (targetRpm == 0) {
                    targetRpm = NEAR_RPM;
                    targetHood = HOOD_MIN;
                } else if (targetRpm == NEAR_RPM) {
                    targetRpm = FAR_RPM;
                    targetHood = HOOD_MAX;
                } else {
                    targetRpm = 0;
                }
            }

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                targetHood += 0.05;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                targetHood -= 0.05;
            }
*/

            // clip our targetRpm .. do this LAST after all command processing
            if(targetRpm > MAX_RPM) targetRpm = MAX_RPM;
        }
    }
}
