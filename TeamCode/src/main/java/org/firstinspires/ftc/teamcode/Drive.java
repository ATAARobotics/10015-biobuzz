package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Configurable
public class Drive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;

    public static double DISTANCE_TOLERANCE_LOW = 0.035; // 25mm in DISTANCE_UNITs to target
    public static double DISTANCE_TOLERANCE = 0.010; // 10mm // in DISTANCE_UNITs to target
    public static double ANGLE_TOLERANCE = 1; // in ANGLE_UNITs to target
    public static double TURN_SPEED = 8;
    public static double POWER_INPUT = 2;
    public static double DEAD_ZONE = 0.1;
    public static final double ANGLE_TWEAK = 0;
    public static double TURBO_FAST_SPEED = 1.0;
    public static double TURBO_SLOW_SPEED = 0.75;

    public static double STATIC_F_SENSITIVE = 0.005; //0.001; //0.04;
    // jan 15, 2025: re-tuned these with the higher "careful_pid"
    // jan 20, re-tuned these with "StaticFriction" op-mode
    public static double STATIC_F_FORWARD = 0.04; // 0.09;
    public static double STATIC_F_STRAFE = 0.08; // 0.15;

    //public static double LINEAR_SCALAR = 1.018;
    //public static double ANGULAR_SCALAR = 0.995;

    MecanumDrive drivebase;
    //SparkFunOTOS otos;
    GoBildaPinpointDriver pinpoint;

    //double current_left_distance = 0.0; // in millimeters
    //Rev2mDistanceSensor dist_left;
    //MovingAverage dist_left_avg;  // filtered, in millimeters
    //double current_right_distance = 0.0; // in millimeters
    //Rev2mDistanceSensor dist_right;
    //MovingAverage dist_right_avg;  // filtered, in millimeters

    // inputs into the drivebase, from human or auto
    double forward; // +Fwd/-Rev
    double strafe; // +Right/-Left
    double turn; // +CW/-CCW
    double desired_heading;
    double ff_forward;
    double ff_strafe;
    double resetStart = 0;
    public enum ResetState {Off, Player, NearZone}
    ResetState reseting = ResetState.Off;

    Command parking; //Null if we're not parking

    private final PIDController heading_control;
    public static PIDCoefficients hPID = new PIDCoefficients(0.015,0,0.0003); //adjusted November 1
    //public static PIDCoefficients PID = new PIDCoefficients(50,0,3);
    //public static PIDCoefficients careful_pid = new PIDCoefficients(1.9, 0, 0.2);
    //public static PIDCoefficients careful_pid = new PIDCoefficients(2.0, 0, 0.2);
    // jan 15, 2025 re-tuned this, also new static-f values
    // jan 20, 2025 -- re-tuning with separate forward/strafe PIDs (at 75% and 100% drivebase)
    public static PIDCoefficients forward_pid_careful = new PIDCoefficients(3.0, 0, 0.45);
    public static PIDCoefficients strafe_pid_careful = new PIDCoefficients(3.0, 0, 0.15);

    // jan 20 -- re-tuned at 100% speed
    // jan 23, 2025 -- re-tuning at max-speed
    public static PIDCoefficients forward_pid_quick = new PIDCoefficients(2.0, 0.05, 0.2);
    public static PIDCoefficients strafe_pid_quick = new PIDCoefficients(2.0, 0.05, 0.2);

    public static Motor.ZeroPowerBehavior zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE;
    public DcMotor turretEncoder;

    double current_time;
    public static Pose2D current_position;

    double previous_time;
    Pose2D previous_position;

    double x_velocity;
    double y_velocity;

    boolean isRedAlliance;
    boolean isAuto;
    AprilTagMetadata target;
    public Drive(HardwareMap hardwareMap, boolean isRedAlliance, boolean isAuto) {
        this.isRedAlliance = isRedAlliance;
        this.isAuto = isAuto;
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
        // BIG NOTE: since we still have the "broken" goBilda
        // floodgate switch, we NEED to wrap our motors so that they
        // don't change power "too fast" (you can potentially trigger
        // this by driving forward at full turbo then immediately
        // driving backwards also in turbo .. we've done this at least
        // once Nov 28)

        Motor motor_fl = new FloodMotor(hardwareMap, "fl");//, Motor.GoBILDA.RPM_435);
        motor_fl.setInverted(true);
        motor_fl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_fr = new FloodMotor(hardwareMap, "fr");//, Motor.GoBILDA.RPM_435);
        turretEncoder = motor_fr.motor;
        motor_fr.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_bl = new FloodMotor(hardwareMap, "bl");//, Motor.GoBILDA.RPM_435);
        motor_bl.setInverted(true);
        motor_bl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_br = new FloodMotor(hardwareMap, "br");//, Motor.GoBILDA.RPM_435);
        motor_br.setZeroPowerBehavior(zeroPowerBehavior);
        drivebase = new MecanumDrive(false, motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(1);

        motor_fl.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_fr.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_bl.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_br.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        heading_control = new PIDController(hPID.p,hPID.i,hPID.d);
        heading_control.setTolerance(ANGLE_TOLERANCE, Double.POSITIVE_INFINITY);

        // configure odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Configure the sensor
        //Updated March 31 for v3. old offsets are: 6.65, 0.866,
        pinpoint.setOffsets(5.693, 1.899, DistanceUnit.INCH); // Note: Y is forward, X is right.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD); //These have to match what is in Constants

        pinpoint.resetPosAndIMU();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void stop() {
        drivebase.stop();
    }

    public static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        current_position = pose;
        desired_heading = pose.getHeading(ANGLE_UNIT);
    }

    public Pose2D getPosition() {
        return current_position;
    }


    // TODO: if we want to keep "auto-park" command use PedroPathing to do it
    public class QuickMoveTo extends CommandBase {
        Pose2D target;
        private final PIDController quick_strafe;
        private final PIDController quick_forward;

        public QuickMoveTo(double x, double y, double h, double tolerance) {
            //target = new Pose2D(x, y, h);
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            //XXX FIXME just trying to hack in higher PID for low-tolerance move
            if (tolerance == DISTANCE_TOLERANCE_LOW) {
                quick_strafe = new PIDController(strafe_pid_quick.p * 2.0, strafe_pid_quick.i, strafe_pid_quick.d);
                quick_forward = new PIDController(forward_pid_quick.p * 2.0, forward_pid_quick.i, forward_pid_quick.d);
            } else {
                quick_strafe = new PIDController(strafe_pid_quick.p, strafe_pid_quick.i, strafe_pid_quick.d);
                quick_forward = new PIDController(forward_pid_quick.p, forward_pid_quick.i, forward_pid_quick.d);
            }
            quick_strafe.setTolerance(tolerance);
            quick_forward.setTolerance(tolerance);
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            quick_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
            quick_forward.setSetPoint(target.getY(DISTANCE_UNIT));
            desired_heading = wrapAngle(target.getHeading(ANGLE_UNIT));
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = quick_strafe.calculate(current_position.getX(DISTANCE_UNIT));
            forward = quick_forward.calculate(current_position.getY(DISTANCE_UNIT));

            // our own "static friction" calc
            if (strafe > STATIC_F_SENSITIVE) ff_strafe = STATIC_F_STRAFE;
            if (strafe < -STATIC_F_SENSITIVE) ff_strafe = -STATIC_F_STRAFE;
            if (forward > STATIC_F_SENSITIVE) ff_forward = STATIC_F_FORWARD;
            if (forward < -STATIC_F_SENSITIVE) ff_forward = -STATIC_F_FORWARD;

            strafe += ff_strafe;
            forward += ff_forward;
        }

        @Override
        public boolean isFinished() {
            // check if the target is reached
            return quick_strafe.atSetPoint() && quick_forward.atSetPoint() && heading_control.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            stop();
        }
    }
    public class Park extends QuickMoveTo{
        GamepadEx driver;
        public Park(GamepadEx driver, double x, double y, double heading, double tolerence){
            super(x, y, heading, tolerence);
            this.driver = driver;
        }
        @Override
        public boolean isFinished(){
            return (!driver.isDown(GamepadKeys.Button.B));
        }
        @Override
        public void end(boolean interupted){
            super.end(interupted);
            parking = null;
            desired_heading = -180;
        }
    }
    public Command parkAt (GamepadEx driver, double x, double y, double heading){
        return new Park(driver, x, y, heading, DISTANCE_TOLERANCE);

    }

    // all interaction with gamepads should go through this inner class
    public class HumanInputs extends CommandBase {
        GamepadEx driver;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(Drive.this);

        }

        @Override
        public void execute() {
            // Run wheels in POV mode: use the Right stick to go forward & strafe, the Left stick to rotate left & right.
            if (isRedAlliance) {
                strafe = scaleInputs(driver.getRightX());
                forward = scaleInputs(-driver.getRightY());
            } else {
                strafe = scaleInputs(-driver.getRightX());
                forward = scaleInputs(driver.getRightY());
            }

            double leftX = driver.getLeftX();
            if (Math.abs(leftX) > DEAD_ZONE)
                desired_heading = wrapAngle(desired_heading - TURN_SPEED * leftX);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                desired_heading = isRedAlliance ? 0 : 180;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                desired_heading = isRedAlliance ? 180 : 0;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                desired_heading = isRedAlliance ? 90 : 270;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                desired_heading = isRedAlliance ? 270 : 90;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP) || driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                desired_heading += ANGLE_TWEAK;
            }

            // if we're holding left trigger _currently_, we go to
            // Turbo -- otherwise to non-Turbo
            turbo(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
            if (driver.isDown(GamepadKeys.Button.A)){
                if (reseting == ResetState.Off){
                    resetStart = current_time;
                    reseting = ResetState.Player;
                }
                else {
                    if (current_time - resetStart > 1.0){
                        setPosition(new Pose2D(DistanceUnit.INCH, 7.179, 7.19, AngleUnit.DEGREES, 90));
                        reseting = ResetState.Off;
                    }
                }
            }
            else if (driver.isDown(GamepadKeys.Button.Y)){
                if (reseting == ResetState.Off){
                    resetStart = current_time;
                    reseting = ResetState.NearZone;
                }
                else {
                    if (current_time - resetStart > 1.0){
                        setPosition(new Pose2D(DistanceUnit.INCH, 70.5, 141 - 7.19, AngleUnit.DEGREES, 270));
                        reseting = ResetState.Off;
                    }
                }
            }
            else {
                reseting = ResetState.Off;
            }
        }

        public class DoNothing extends CommandBase {
        }

        public double scaleInputs(double input) {
            if (Math.abs(input) > DEAD_ZONE)
                return Math.pow(Math.abs(input), POWER_INPUT) * Math.signum(input);
            else
                return 0;
        }
    }

    public void turbo(boolean on) {
        if (on) {
            drivebase.setMaxSpeed(TURBO_FAST_SPEED);
            // TODO might want differently-tuned heading-lock PIDs for turbo vs not-turbo
        } else {
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }
    }

    // called ONCE, before any driver or robot inputs
    public void read_sensors(double time) {
        // Get the latest pose, which includes the x and y coordinates, plus the heading angle
        previous_time = current_time;
        current_time = time;
        previous_position = current_position;

        pinpoint.update();
        current_position = pinpoint.getPosition();

        // if we have at least two positions, we can compute our velocity
        if (previous_position != null && (current_time - previous_time) > 0.0) {
            double interval = current_time - previous_time;
            x_velocity = (current_position.getX(DistanceUnit.INCH) - previous_position.getX(DistanceUnit.INCH)) / interval;
            y_velocity = (current_position.getY(DistanceUnit.INCH) - previous_position.getY(DistanceUnit.INCH)) / interval;
        }
    }

    @Override
    public void periodic() {
        // heading lock
        //heading_control.setPID(hPID.p,hPID.i,hPID.d);
        turn = heading_control.calculate(wrapAngle(desired_heading - current_position.getHeading(ANGLE_UNIT)));
        // tell ftclib its inputs
        if (!isAuto) {
            drivebase.driveFieldCentric(strafe, forward, turn, current_position.getHeading(ANGLE_UNIT), false);
        }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.logBoth("position-x", current_position.getX(DISTANCE_UNIT));
        telem.logBoth("position-y", current_position.getY(DISTANCE_UNIT));
        telem.logBoth("position-heading", current_position.getHeading(AngleUnit.DEGREES));

        double targetX = target.distanceUnit.toInches(target.fieldPosition.get(1)) + 70.5;
        double targetY = target.distanceUnit.toInches(target.fieldPosition.get(0)) + 70.5;
        telem.log("target-x", targetX);
        telem.log("target-y", targetY);
        telem.log("current-heading", current_position.getHeading(ANGLE_UNIT));
        telem.log("desired-heading", desired_heading);
        telem.log("velocity-x", x_velocity);
        telem.log("velocity-y", y_velocity);

       // telem.log("strafe", strafe);
       // telem.log("forward", forward);
       // telem.log("strafe_ff", ff_strafe);
       // telem.log("forward_ff", ff_forward);
        //telem.log("turn", turn);
    }
}
