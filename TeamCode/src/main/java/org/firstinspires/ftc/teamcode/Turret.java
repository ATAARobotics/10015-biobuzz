package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Config
public class Turret extends SubsystemBase {
    private final CRServo servo1, servo2;
    AnalogInput encoder;

    public PIDController turretHeadingControl;
    public double servoPower, currentTurretAngle;
    private double servoAngle;
    private double servoDelta = Double.NaN;
    private double lastServoAngle = Double.NaN;
    private double resetAngle;
    private int servoTurnCount;
    double joystickAngle;
    double operatorOffset = 0;

    public boolean haveAprilLock;
    public double april_bearing;
    public double april_distance;

    private static final double GEAR_RATIO = 0.8; // 1 servo rotation equals 0.8 turret rotations
    // tuned december 11, bare servos for PID, attach turret for F
    //public static double turretP = 0.004, turretI = 0.06, turretD = 0.0005, turretF = 0.015;
    // tuned dec 22 from first principals
    //////public static double turretP = 0.003, turretI = 0.00, turretD = 0.0, turretF = 0.07;
    // (and again)
    public static double turretP = 0.0045, turretI = 0.00, turretD = 0.0002, turretF = 0.07;
    public static double TURRET_TOLERANCE = 2.5; // in degrees
    public static double TURRET_TWEAK = 2;
    public double apriltag_heading, robot_heading;

    public enum HeadingLockMode { Trig, Camera, Off, Both }
    private HeadingLockMode mode = HeadingLockMode.Off;

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;
    public AprilTagMetadata target;


//    private static FileWriter writer;

    public Turret(HardwareMap hardwareMap, boolean isRedAlliance) {
        target = AprilTagGameDatabase.getDecodeTagLibrary().lookupTag(isRedAlliance ? 24 : 20);
        // both servos must always run in the same direction
        servo1 = new CRServo(hardwareMap, "left_turret");
        servo2 = new CRServo(hardwareMap, "right_turret");
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
//        try { writer = new FileWriter("/sdcard/FIRST/axon_debug.txt"); } catch (IOException e) { e.printStackTrace(); }
        reset();

        //AprilTagLibrary decode_tags = ;
        // game manual says april tag family is 36h11
        april_tags = new AprilTagProcessor.Builder()
                //.setTagLibrary(decode_tags)
                .setDrawTagID(true)
                .setDrawTagOutline(false)//true)
                .setDrawAxes(false)//true)
                .setDrawCubeProjection(false)//true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"elp"))
                .addProcessor(april_tags)
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

    }

    public void faceFieldAngle(double fieldAngleDeg) {
        // Robot-relative angle: where turret must point relative to robot frame
        double robotRelative = wrapAngle(fieldAngleDeg + operatorOffset - robot_heading);

        // Reuse existing robot-relative method
        faceRobotAngle(robotRelative);
    }

    public void faceRobotAngle(double angle) {
        // +/- 90 is easier to see obvious issues with turret angle/tracking
        angle = Math.max(-90, Math.min(angle, 90));
        //angle = Math.max(-135, Math.min(angle, 135));
        turretHeadingControl.setSetPoint(angle);
    }

    public boolean atTargetAngle() {
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
        currentTurretAngle = 0;
        lastServoAngle = 0;
        servoTurnCount = 0;
        servo1.stop();
        servo2.stop();
        angleReset();
        faceRobotAngle(0);
        mode = HeadingLockMode.Off;
    }

    public void autoLock() {
        mode = HeadingLockMode.Both;
        //mode = HeadingLockMode.Camera;
    }

    public void noLock() {
        mode = HeadingLockMode.Off;
    }

    public void angleReset() {
        resetAngle = getServoAngle();
    }

    public double getServoAngle() {
        // Read analog voltage, convert to degrees
        return encoder.getVoltage()/3.3 * 360;
    }

    @Override
    public void periodic() {
        // do some math based on which "mode" we're in
        if (mode == HeadingLockMode.Off)
            faceRobotAngle(joystickAngle);
        if (mode == HeadingLockMode.Trig)
            faceFieldAngle(apriltag_heading);
        if (mode == HeadingLockMode.Camera) {
            haveAprilLock = aprilTagLock();
        }
        if (mode == HeadingLockMode.Both){
            haveAprilLock = aprilTagLock();
            if(! haveAprilLock){
                faceFieldAngle(apriltag_heading);
            }
        }

        // compute where the servos are, and conclude where the turret is
        servoAngle = getServoAngle() - resetAngle;
        servoDelta = lastServoAngle - servoAngle;
        lastServoAngle = servoAngle;

        // did we just "wrap around"?
        if (servoDelta < -180) servoTurnCount--;
        if (servoDelta > 180) servoTurnCount++;

        currentTurretAngle = (servoTurnCount * 360 + servoAngle)*GEAR_RATIO;

        turretHeadingControl.setPID(turretP, turretI, turretD);

        servoPower = turretHeadingControl.calculate(currentTurretAngle) + turretF*Math.signum(turretHeadingControl.getPositionError());
        if (servoPower > 1.0) servoPower = 1.0;
        if (servoPower < -1.0) servoPower = -1.0;
        servo1.set(servoPower);
        servo2.set(servoPower);
//        try { writer.write(servoAngle+"\t"+currentTurretAngle+"\t"+delta+"\n"); } catch (IOException e) { e.printStackTrace(); }
    }

    private static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }
/*
    public boolean isFinished() {
        // check if the target is reached
        return turretHeadingControl.atSetPoint();
    }
 */
    public void stop() {
        servo1.stop();
        servo2.stop();
//        try { writer.close(); } catch (IOException e) { e.printStackTrace(); }
    }

    public void addTelemetry(HyperTelemetry telem) {
        telem.log("turret-current-angle", currentTurretAngle);
        telem.log("turret-target-angle", turretHeadingControl.getSetPoint());
        telem.log("turret-power", servoPower);
        telem.log("turret-error", turretHeadingControl.getPositionError());
        telem.log("turret-joystick", joystickAngle);
        telem.log("turret-servo-angle", servoAngle);
        telem.log("turret-last-servo-angle", lastServoAngle);
        telem.log("turret-servo-turn-count", servoTurnCount);
        telem.log("turrent-servo-last", lastServoAngle);
        telem.log("turret-servo-delta", servoDelta);
        telem.log("turret-april-bearing", april_bearing);
        telem.log("turret-april-distance", april_distance);
        telem.log("turret-april-lock", haveAprilLock);
        telem.log("turret-april-mode", mode);
        telem.log("turret-april-fps", portal.getFps());

        telem.logDrivers("Heading Lock Mode", mode);
        telem.logDrivers("Turret Current Angle", currentTurretAngle);
        telem.logDrivers("Turret Target Angle ", turretHeadingControl.getSetPoint());
        telem.logDrivers("Turret Power", servoPower);
        telem.logDrivers("Turret Angle Error", turretHeadingControl.getPositionError());
        telem.logDrivers("Joystick Angle", joystickAngle);
        telem.logDrivers("Turret Tweak", operatorOffset);
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
            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (mode == HeadingLockMode.Off) {
                    mode = HeadingLockMode.Both;
                } else {
                    mode = HeadingLockMode.Off;
                    joystickAngle = 0;
                    // reset operator desired angle when switching mode
                }
            }

            // decide what to do based on sensors and human inputs from controller

            // face turret the same way the joystick is facing ... and
            // let the operator tweak the angle with DPAD
            double rx = -operator.getRightX();
            double ry = -operator.getRightY();

            // only do the joystick control if it has moved "a lot" (1.0 is slammed)
            if (Math.hypot(rx, ry) > 0.8) {
                joystickAngle = Math.toDegrees(Math.atan2(rx, ry));
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                operatorOffset += TURRET_TWEAK;
                //joystickAngle = 90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                operatorOffset -= TURRET_TWEAK;
                //joystickAngle = -90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                operatorOffset = 0;
            }
        }
    }

    private boolean aprilTagLock() {
        List<AprilTagDetection> detections = april_tags.getFreshDetections();
        if (detections == null) {
            // there are no _fresh_ detections, but we may have had a
            // recent lock
            return haveAprilLock;
        }

        for (AprilTagDetection tag : detections) {
            if (tag.id == target.id){
                //drive.april_bearing = drive.getPosition().getHeading(AngleUnit.DEGREES) + tag.ftcPose.bearing;
                faceRobotAngle(tag.ftcPose.bearing + currentTurretAngle);
                april_bearing = tag.ftcPose.bearing;
                april_distance = tag.ftcPose.range;
                //telem.log("bearing", tag.ftcPose.bearing);
                //range(distance)is in inches, maybe convert to centi
                //telem.log("distance to april tag, inches", tag.ftcPose.range);
                return true;
            }
        }
        return false;
    }
}
