package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class Turret extends SubsystemBase {
    private final CRServo servo1, servo2;
    AnalogInput encoder0;
    AnalogInput encoder1;
    DcMotor revEncoder;
    public Servo motifLight;

    double PINK = 0.71;

    double voltage0;
    double voltage1;
    double time;
    public double ticks;

    // 88.67mm
    private final double CAMERA_TO_TURRET_CENTER_INCHES = 3.491;

    double obeliskHeading;

    public boolean haveAprilLock;
    public double lastAprilLock;
    public double aprilBearing;
    public double angleAdjust;
    public double aprilDistance;
    public double aprilFloorDistance;

    // this is the total gear-ratio from the servos to the (belted) turret.
    // that is, one servo rotation equals 0.9153 turret rotations
    private static final double SERVO_BELT_RATIO = 0.9153;  // 54 / 59

    // 3:1 then 90:295
    
    // tuned december 11, bare servos for PID, attach turret for F
    //public static double turretP = 0.004, turretI = 0.06, turretD = 0.0005, turretF = 0.015;
    // tuned dec 22 from first principals
    /// ///public static double turretP = 0.003, turretI = 0.00, turretD = 0.0, turretF = 0.07;
    // (and again)
    // april 13, both servos definitely working.
    public static double turretP = 0.005, turretI = 0.04, turretD = 0.0008, turretF = 0.06;
    public static double TURRET_TOLERANCE = 4; // in degrees
    public static double TURRET_TWEAK = 3;
    public static boolean ALWAYS_LOCK = false;
    public double targetHeading;  // from geometry via RobotBaseOp
    public double robotHeading;

    public enum HeadingLockMode {Trig, Camera, Off, Both, Obelisk}

    private HeadingLockMode mode = HeadingLockMode.Off;
    private HeadingLockMode modeOverride = HeadingLockMode.Trig;
    private boolean modeJustChanged = false;


    // servo -> turret angle and wrapping issues and PID control of
    // turret heading (controlled by targetTurretAngle)
    public PIDController turretHeadingControl;
    public double servoPower;
    public double currentTurretAngle;
    public double targetTurretAngle;
    public double servoAngle;
    public double servoReset;
    double joystickAngle;
    double operatorOffset = 0;

    public enum MovementMode {Left, Middle, Right};
    private MovementMode movement = MovementMode.Middle;

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;
    public AprilTagMetadata target;
    int pattern = -1;
    boolean isAuto = false;


//    private static FileWriter writer;

    public Turret(HardwareMap hardwareMap, boolean isRedAlliance, boolean isAuto, DcMotor rev) {
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
        this.isAuto = isAuto;
        // both servos must always run in the same direction
        servo1 = new CRServo(hardwareMap, "left_turret");
        servo2 = new CRServo(hardwareMap, "right_turret");
        encoder0 = hardwareMap.get(AnalogInput.class, "left_encoder");
        encoder1 = hardwareMap.get(AnalogInput.class, "right_encoder");
        // the turret encoder is plugged into drivebase port 0, the front-right motor
        //revEncoder = hardwareMap.dcMotor.get("fr");
        revEncoder = rev;
        motifLight = hardwareMap.get(Servo.class, "light");

        turretHeadingControl = new PIDController(turretP, turretI, turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
//        try { writer = new FileWriter("/sdcard/FIRST/axon_debug.txt"); } catch (IOException e) { e.printStackTrace(); }
        reset();

        //AprilTagLibrary decode_tags = ;
        // game manual says april tag family is 36h11
      /*  april_tags = new AprilTagProcessor.Builder()
                //.setTagLibrary(decode_tags)
                .setDrawTagID(true)
                .setDrawTagOutline(false)//true)
                .setDrawAxes(false)//true)
                .setDrawCubeProjection(false)//true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "elp"))
                .addProcessor(april_tags)
                .setCameraResolution(new Size(1024, 768))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();*/
    }

    public void faceFieldAngle(double fieldAngleDeg) {
        // Robot-relative angle: where turret must point relative to robot frame
        //double robotRelative = wrapAngle(fieldAngleDeg + operatorOffset - robotHeading);
        double robotRelative = fieldAngleDeg + operatorOffset - robotHeading;
	robotRelative = wrapAngle(robotRelative);

        // Reuse existing robot-relative method
        faceRobotAngle(robotRelative);
    }

    public void faceRobotAngle(double angle) {
	double maxAngle = 160;
	if (angle < -maxAngle) angle = -maxAngle;
	if (angle > maxAngle) angle = maxAngle;
	targetTurretAngle = angle;
	// "set point" is always zero and we compute the error ourselves
        turretHeadingControl.setSetPoint(0.0);
    }

    // convert a 0->360 angle to range -180 -> +180 where 0 is robot-forward
    public double convertAngleRobotRelative(double angle) {
	// make sure angle isn't crazy
	if (angle > 360) {
	    angle -= 360.0;
	}
	if (angle < 0) {
	    angle = 360 - angle;
	}

	// note: might be better for wire management for this to be
	// "asymmetric", that is +90 to -260 for example
	
	// so if angle is "more than straight backwards" we make it a
	// negative angle (i.e. 190 becomes -170)
	if (angle > 180) {
	    angle = -180 - (180 - angle);
	}
	return angle;
    }

    public void faceObelisk(double angle) {
        mode = HeadingLockMode.Obelisk;
        obeliskHeading = angle;
    }

    public boolean atTargetAngle() {
        if (modeJustChanged) return false;
        return turretHeadingControl.atSetPoint();
    }

    /// trying to re-tun december 22
    // tolerate 0.5
    // d = 0.0005
    // f = 0.07
    // i = 0.0
    // p = 0.005

    /// decent behavior at -90 and +90 but jiggled around a lot at about "0"
    /// looked like P oscillations, but the servo was right near it's "flip" angle
    /// (and then it did the freak-out thing and went all the way around)
    // pidf = 0.0055, 0, 0.0005, 0.07
    // f "just below moving" = 0.11
    // pidf = 0.003, 0, 0.07, 0.0   <-- seems pretty good?
    public void reset() {
        servo1.stop();
        servo2.stop();
        currentTurretAngle = 0;
	targetTurretAngle = 0;
	movement = MovementMode.Middle;
	
        joystickAngle = 0;
        revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

	servoReset = getServoAngle();
        faceRobotAngle(0);
        mode = HeadingLockMode.Off;
    }

    public void autoLock() {
        if (mode != modeOverride) {
            modeJustChanged = true;
        }
        mode = modeOverride;
        //   mode = HeadingLockMode.Camera;
    }

    public void toggleOverride() {
        if (modeOverride == HeadingLockMode.Both) {
            modeOverride = HeadingLockMode.Trig;
        } else if (modeOverride == HeadingLockMode.Trig) {
            modeOverride = HeadingLockMode.Both;
        }
    }

    public void noLock() {
        mode = HeadingLockMode.Off;
    }

    public double getServoAngle() {
        // Read analog voltage, convert to degrees
        return 360 - (voltage0 / 3.3 * 360);
    }

    public double getTurretAngle() {
	// we want as much range as possible, but the absolute
	// encoders on the Axons "flip over" on every revolution, and
	// one revolution of the servo is less than one revolution of
	// the turret.

	// this is solved by remembering "which way" we're moving near
	// the ends of the ranges so that we can "tack on" the extra
	// to the correct direction -- this wouldn't work for
	// multi-revolution turret, but we only go about +/- 180 (but
	// can overshoot a bit etc and don't want to get confused). We
	// had problems trying to count "complete revolutions" of the
	// individual servos because there's a decent amount of noise
	// in the encoder values.

	double servoAngle = getServoAngle();
	double angle = 0.0;
	switch(movement) {
	case Middle:
	    if (servoReset < 180) {
		if (servoAngle > (servoReset + 180))
		    angle = -(servoReset + (360 - servoAngle));
		else
		    angle = servoAngle - servoReset;
	    } else {
		/// todo don't we need an if/else in here too?
		angle = servoAngle - servoReset;
	    }
	    break;

	case Left:
	    if (servoAngle > servoReset) {
		double extra = 360 - servoAngle;
		angle = -servoReset - extra;
	    } else {
		angle = servoAngle - servoReset;
	    }
	    break;

	case Right:
	    if (servoAngle < servoReset) {
		angle = (360 - servoReset) + servoAngle;
	    } else {
		angle = servoAngle - servoReset;
	    }
	    break;
	}

	// account for gear ratios (above we basically measured the
	// "servo angle")
	return angle * SERVO_BELT_RATIO;
    }

    public double getOtherServoAngle() {
        // Read analog voltage, convert to degrees
        return voltage1 / 3.3 * 360;
    }

    public void read_sensors(double time) {
        this.time = time;
        this.ticks = revEncoder.getCurrentPosition();
        voltage0 = encoder0.getVoltage();
        voltage1 = encoder1.getVoltage();
        currentTurretAngle = getTurretAngle();
    }

    @Override
    public void periodic() {
        processAprilTags();
        // do some math based on which "mode" we're in

        if (mode == HeadingLockMode.Obelisk)
            faceFieldAngle(obeliskHeading);

	if (mode == HeadingLockMode.Off)
	    faceRobotAngle(joystickAngle);

        if (mode == HeadingLockMode.Trig)
            faceFieldAngle(targetHeading);
        if (mode == HeadingLockMode.Camera) {
            haveAprilLock = aprilTagLock();
        }
        if (mode == HeadingLockMode.Both) {
            haveAprilLock = aprilTagLock();
            if (!haveAprilLock) {
                faceFieldAngle(targetHeading);
            }
        }
        // TEMP: always face our april-tag
	if (ALWAYS_LOCK) {
	    faceFieldAngle(targetHeading);
	}

       // maybe only in auto?
        if (isAuto) {
            // show the operator what motif we detected
            if (pattern < 0) {
                motifLight.setPosition(0.0);
            } else {
                // TODO: change color depending on motif pattern
                motifLight.setPosition(PINK);
            }
        }

        turretHeadingControl.setPID(turretP, turretI, turretD);

	double error = wrapAngle(currentTurretAngle) - targetTurretAngle;
        servoPower = -(turretHeadingControl.calculate(error) + turretF * Math.signum(turretHeadingControl.getPositionError()));
	if (currentTurretAngle < -90) movement = MovementMode.Left;
	else if (currentTurretAngle > 90) movement = MovementMode.Right;
	else movement = MovementMode.Middle;

        if (servoPower > 1.0) servoPower = 1.0;
        if (servoPower < -1.0) servoPower = -1.0;
        servo1.set(servoPower);
        servo2.set(servoPower);
//        try { writer.write(servoAngle+"\t"+currentTurretAngle+"\t"+delta+"\n"); } catch (IOException e) { e.printStackTrace(); }
        modeJustChanged = false;
    }

    public boolean isLocked(double time) {
        if (haveAprilLock || (time - lastAprilLock) < 0.3) {
            return true;
        }
        return false;
    }

    private static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }

    public void stop() {
        servo1.stop();
        servo2.stop();
//        try { writer.close(); } catch (IOException e) { e.printStackTrace(); }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("turret-current-angle", currentTurretAngle);
        telem.log("turret-voltage0", voltage0);
        telem.log("turret-voltage1", voltage1);
        telem.log("turret-target-angle", targetTurretAngle);
        telem.log("turret-power", servoPower);
        telem.log("turret-error", turretHeadingControl.getPositionError());
        telem.log("turret-joystick", joystickAngle);
        telem.log("turret-servo-angle0", getServoAngle());
        telem.log("turret-servo-angle1", getOtherServoAngle());
	telem.log("turret-movement", movement);
        telem.log("turret-servo-reset", servoReset);
        telem.log("turret-april-bearing", aprilBearing);
        telem.log("turret-angle-adjust", angleAdjust);
        telem.log("turret-april-distance", aprilDistance);
        telem.log("turret-april-lock", haveAprilLock);
        telem.log("turret-last-april", lastAprilLock);
        telem.log("turret-april-mode", mode);
        telem.log("turret-obelisk", pattern);
        telem.log("turret-operator-offset", operatorOffset);

        String logPattern = "unknown";
        if (pattern == 0) {
            logPattern = "G P P";
        } else if (pattern == 1) {
            logPattern = "P G P";
        } else if (pattern == 2) {
            logPattern = "P P G";
        }
	/*
        telem.logDrivers("Heading Lock Mode", mode);
        telem.logDrivers("Obelisk", logPattern);
        telem.logDrivers("Turret Current Angle", currentTurretAngle);
        telem.logDrivers("Turret Target Angle ", targetTurretAngle);
	telem.logDrivers("Servo Reset", servoReset);
	telem.logDrivers("Servo Angle", getServoAngle());
        telem.logDrivers("Turret Power", servoPower);
        telem.logDrivers("Turret Angle Error", turretHeadingControl.getPositionError());
        telem.logDrivers("Joystick Angle", joystickAngle);
        telem.logDrivers("Turret Tweak", operatorOffset);
	*/
    }


    public class HumanInputs extends CommandBase {
        GamepadEx driver;
        GamepadEx operator;

        public HumanInputs(GamepadEx operator, GamepadEx driver) {
            this.operator = operator;
            this.driver = driver;
            addRequirements(Turret.this);
        }

        @Override
        public void execute() {
            // for ease-of-use we have just two modes:
            // - "use april tag if available, else trig"
            // - "off (lock at 0)"
           /* if (operator.wasJustPressed(GamepadKeys.Button.B)) {
                toggleOverride();

            }*/

            // decide what to do based on sensors and human inputs from controller

            // face turret the same way the joystick is facing ... and
            // let the operator tweak the angle with DPAD
            double rx = -operator.getRightX();
            double ry = -operator.getRightY();

            // only do the joystick control if it has moved "a lot" (1.0 is slammed)
            if (Math.hypot(rx, ry) > 0.8) {
		// this returns -180 through +180
                joystickAngle = Math.toDegrees(Math.atan2(rx, ry));
		// (if needed, can convert it to 0 .. 360)
		//joystickAngle += 180.0;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                operatorOffset -= TURRET_TWEAK;
                //joystickAngle = -90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                operatorOffset += TURRET_TWEAK;
                //joystickAngle = 90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                //operatorOffset = 0;
		//joystickAngle = 0;
            }
        }
    }

    private boolean aprilTagLock() {
        if (time - lastAprilLock < 0.3) {
            faceRobotAngle(aprilBearing + currentTurretAngle - angleAdjust);
            return true;
        }
        return false;
    }

    private void processAprilTags() {
        if (april_tags == null){
            return;
        }
        List<AprilTagDetection> detections = april_tags.getFreshDetections();
        if (detections == null) {
            // there are no _fresh_ detections, but we may have had a
            // recent lock
            return;
        }

        for (AprilTagDetection tag : detections) {
            if (tag.id == target.id) {
                //drive.aprilBearing = drive.getPosition().getHeading(AngleUnit.DEGREES) + tag.ftcPose.bearing;
                angleAdjust = Math.atan(tag.ftcPose.range / CAMERA_TO_TURRET_CENTER_INCHES);
                aprilBearing = tag.ftcPose.bearing;
                aprilDistance = tag.ftcPose.range;
                lastAprilLock = time;
            }
            else if (tag.id == 21 && pattern == -1) {
                pattern = 0;
            }
            else if (tag.id == 22 && pattern == -1) {
                pattern = 1;
            }
            else if (tag.id == 23 && pattern == -1) {
                pattern = 2;
            }
        }
    }
}
