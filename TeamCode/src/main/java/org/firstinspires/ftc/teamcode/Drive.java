package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Config
public class Drive extends SubsystemBase {
    static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    static final DistanceUnit DISTANCE_UNIT = DistanceUnit.METER;

    public static double DISTANCE_TOLERANCE_LOW = 0.035; // 25mm in DISTANCE_UNITs to target
    public static double DISTANCE_TOLERANCE = 0.010; // 10mm // in DISTANCE_UNITs to target
    public static double ANGLE_TOLERANCE = 1; // in ANGLE_UNITs to target
    public static double TURN_SPEED = 12;
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
    public double apriltag_heading;
    double ff_forward;
    double ff_strafe;
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

    double current_time;
    public static Pose2D current_position;

    double previous_time;
    //Pose2D previous_position;

    //double x_velocity;
    //double y_velocity;

    boolean isRedAlliance;
    AprilTagMetadata target;
    public Drive(HardwareMap hardwareMap, boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
        // BIG NOTE: since we still have the "broken" goBilda
        // floodgate switch, we NEED to wrap our motors so that they
        // don't change power "too fast" (you can potentially trigger
        // this by driving forward at full turbo then immediately
        // driving backwards also in turbo .. we've done this at least
        // once Nov 28)

        Motor motor_fl = new FloodMotor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
        motor_fl.setInverted(true);
        motor_fl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_fr = new FloodMotor(hardwareMap, "fr", Motor.GoBILDA.RPM_435);
        motor_fr.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_bl = new FloodMotor(hardwareMap, "bl", Motor.GoBILDA.RPM_435);
        motor_bl.setInverted(true);
        motor_bl.setZeroPowerBehavior(zeroPowerBehavior);
        Motor motor_br = new FloodMotor(hardwareMap, "br", Motor.GoBILDA.RPM_435);
        motor_br.setZeroPowerBehavior(zeroPowerBehavior);
        drivebase = new MecanumDrive(false, motor_fl, motor_fr, motor_bl, motor_br);
        drivebase.setMaxSpeed(1);

        heading_control = new PIDController(hPID.p,hPID.i,hPID.d);
        heading_control.setTolerance(ANGLE_TOLERANCE, Double.POSITIVE_INFINITY);

        // configure odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Configure the sensor
        pinpoint.setOffsets(22, -169, DistanceUnit.MM); // Note: Y is forward, X is right
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        /*otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DISTANCE_UNIT);
        otos.setAngularUnit(ANGLE_UNIT);
        // 169.91mm from back of arm to "center of robot"
        // theory: our "center of drivebase" is not actually where the
        // robot rotates around .. and so this offset isn't actually
        // correct, causing a bit more drive when we "turn and drive"
        otos.setOffset(new Pose2D(0, 0.0466, 0));
        // notes:
        // (above offset is the offset from the exact _center_ of the robot)
        // from CAD, December 6:
        //   - OTOS is 130.37mm from front of robot
        //   - OTOS is 154.35mm from side of robot (it's centered, so from either side)
        //   - OTOS offset is 46.635mm from exact center

        otos.setLinearScalar(LINEAR_SCALAR);
        otos.setAngularScalar(ANGULAR_SCALAR);
        otos.calibrateImu();
*/
        // distance sensor
        // TODO: more efficient if plugged to control-hub (not expansion)?
        /*
        dist_left = hardwareMap.get(Rev2mDistanceSensor.class, "dist_left");
        dist_left_avg = new MovingAverage(20);
        dist_right = hardwareMap.get(Rev2mDistanceSensor.class, "dist_right");
        dist_right_avg = new MovingAverage(20);
         */
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        //otos.resetTracking();
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
        //otos.setPosition(pose);
        pinpoint.setPosition(pose);
        //TODO FIXME
//        previous_position = current_position;
        current_position = pose;
        desired_heading = pose.getHeading(ANGLE_UNIT);
    }

    public Pose2D getPosition() {
        return current_position;
    }

    public Command moveCarefully(double x, double y, double heading) {
        return new CarefulMoveTo(x, y, heading);
    }

    public Command moveQuickly(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading, DISTANCE_TOLERANCE);
    }
    public Command moveLowThreshold(double x, double y, double heading) {
        return new QuickMoveTo(x, y, heading, DISTANCE_TOLERANCE_LOW);
    }

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

            // careful, take out for production FIXME TODO
            /*
            if (false) {
                otos.setLinearScalar(LINEAR_SCALAR);
                otos.setAngularScalar(ANGULAR_SCALAR);
            }
            */

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

    public class CarefulMoveTo extends CommandBase {
        Pose2D target;
        private PIDController careful_strafe;
        private PIDController careful_forward;

        public CarefulMoveTo(double x, double y, double h) {
            //System.out.println("x="+(x-current_position.x)+" y="+(y-current_position.y)+" h="+(h-current_position.h));
            target = new Pose2D(DISTANCE_UNIT, x, y, ANGLE_UNIT, h);
            careful_strafe = new PIDController(strafe_pid_careful.p, strafe_pid_careful.i, strafe_pid_careful.d);
            careful_forward = new PIDController(forward_pid_careful.p, forward_pid_careful.i, forward_pid_careful.d);
            careful_strafe.setTolerance(DISTANCE_TOLERANCE);
            careful_forward.setTolerance(DISTANCE_TOLERANCE);
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            careful_strafe.setSetPoint(target.getX(DISTANCE_UNIT));
            careful_forward.setSetPoint(target.getY(DISTANCE_UNIT));
            desired_heading = wrapAngle(target.getHeading(ANGLE_UNIT));

            // careful, take out for production FIXME TODO
            /*if (false) {
                otos.setLinearScalar(LINEAR_SCALAR);
                otos.setAngularScalar(ANGULAR_SCALAR);
            }

             */
            drivebase.setMaxSpeed(TURBO_SLOW_SPEED);
        }

        @Override
        public void execute() {
            // compute the direction vector relatively to the robot coordinates
            strafe = careful_strafe.calculate(current_position.getX(DISTANCE_UNIT));
            forward = careful_forward.calculate(current_position.getY(DISTANCE_UNIT));

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
            return careful_strafe.atSetPoint() && careful_forward.atSetPoint() && heading_control.atSetPoint();
        }

        @Override
        public void end(boolean interrupted) {
            strafe = 0;
            forward = 0;
            stop();
        }
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
                strafe = scaleInputs(-driver.getRightY());
                forward = scaleInputs(-driver.getRightX());
            } else {
                strafe = scaleInputs(driver.getRightY());
                forward = scaleInputs(driver.getRightX());
            }

            double leftX = driver.getLeftX();
            if (Math.abs(leftX) > DEAD_ZONE)
                desired_heading = wrapAngle(desired_heading - TURN_SPEED * leftX);
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                desired_heading = isRedAlliance ? -90 : 90;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                desired_heading = isRedAlliance ? 90 : -90;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                desired_heading = isRedAlliance ? 0 : 180;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                desired_heading = isRedAlliance ? 180 : 0;
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP) || driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                desired_heading += ANGLE_TWEAK;
            }
            //if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            //desired_heading = apriltag_heading;

             // Anjalika wants "turbo" mode ... so if we're holding
            // left trigger _currently_, we go to Turbo -- otherwise
            // to non-Turbo
            turbo(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
            if (driver.isDown(GamepadKeys.Button.B) && isRedAlliance && parking == null){
                parking = parkAt(driver, -0.835, -0.95, -180);
                CommandScheduler.getInstance().schedule(parking);
            }
            if (driver.isDown(GamepadKeys.Button.B) && !isRedAlliance && parking == null) {
                parking = parkAt(driver, 0.835, -0.95, -180);
                CommandScheduler.getInstance().schedule(parking);
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
        //previous_position = current_position;
        //current_position = otos.getPosition();
        pinpoint.update();
        current_position = pinpoint.getPosition();

        apriltag_heading = Math.toDegrees(Math.atan2(
                current_position.getX(DistanceUnit.METER) - target.distanceUnit.toMeters(target.fieldPosition.get(1)),
                -current_position.getY(DistanceUnit.METER) - target.distanceUnit.toMeters(target.fieldPosition.get(0))));
        /*
        current_left_distance= dist_left.getDistance(DistanceUnit.INCH);
        dist_left_avg.add_sample(current_left_distance);
        current_right_distance= dist_right.getDistance(DistanceUnit.INCH);
        dist_right_avg.add_sample(current_right_distance);
        */
        // if we have at least two positions, we can compute our velocity
        /*
        if (previous_position != null && (current_time - previous_time) > 0.0) {
            double interval = current_time - previous_time;
            x_velocity = (current_position.x - previous_position.x) / interval;
            y_velocity = (current_position.y - previous_position.y) / interval;
        }
        */
    }

    @Override
    public void periodic() {
        // heading lock
        //heading_control.setPID(hPID.p,hPID.i,hPID.d);
        turn = heading_control.calculate(wrapAngle(desired_heading - current_position.getHeading(ANGLE_UNIT)));
        // tell ftclib its inputs
        drivebase.driveFieldCentric(strafe, forward, turn, current_position.getHeading(ANGLE_UNIT), false);
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("position-x", current_position.getX(DISTANCE_UNIT));
        telem.log("position-y", current_position.getY(DISTANCE_UNIT));
        //telem.log("position-x-cm", current_position.getX(DistanceUnit.CM));
      //  telem.log("position-y-cm", current_position.getY(DistanceUnit.CM));
        //telem.log("target-x", fixme);
        //telem.log("target-y", fixme);
        telem.log("current-heading", current_position.getHeading(ANGLE_UNIT));
        telem.log("desired-heading", desired_heading);

       // telem.log("strafe", strafe);
       // telem.log("forward", forward);
       // telem.log("strafe_ff", ff_strafe);
       // telem.log("forward_ff", ff_forward);
        //telem.log("turn", turn);
        /*
        telem.log("dist_left_current", current_left_distance);
        telem.log("dist_left_avg", dist_left_avg.current_value());
        telem.log("dist_right_current", current_right_distance);
        telem.log("dist_right_avg", dist_right_avg.current_value());
        */
    }
}
