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
    public double lastServoAngle, servoPower, currentTurretAngle;
    private double servoAngle;
    private double resetAngle;
    private int servoTurnCount;
    double joystickAngle;

    private static final double GEAR_RATIO = 0.8; // 1 servo rotation equals 0.8 turret rotations
    public static double turretP = 0.005, turretI = 0.04, turretD = 0.0003, turretF = 0.055; // Tuned 2025.11.23 with goBILDA 6V Servo Power Injector
    public static double TURRET_TOLERANCE = 1.0; // in degrees
    public double apriltag_heading, robot_heading;

    public enum HeadingLockMode { Trig, Camera, Off, Both }
    private HeadingLockMode mode = HeadingLockMode.Off;

    // prototyping with some AprilTags, Sept 15
    AprilTagProcessor april_tags;
    VisionPortal portal;
    AprilTagMetadata target;


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
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
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
        double robotRelative = wrapAngle(fieldAngleDeg - robot_heading);

        // Reuse existing robot-relative method
        faceRobotAngle(robotRelative);
    }

    public void faceRobotAngle(double angle) {
        angle = Math.max(-135, Math.min(angle, 135));
        turretHeadingControl.setSetPoint(angle);
    }

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

    public void angleReset() {
        resetAngle = getServoAngle();
    }

    public double getServoAngle() {
        // Read analog voltage, convert to degrees
        return encoder.getVoltage()/3.3 * 360;
    }

    @Override
    public void periodic() {
        servoAngle = getServoAngle() - resetAngle;
        double delta = (servoAngle - lastServoAngle)/turretHeadingControl.getPeriod();
        lastServoAngle = servoAngle;

        // Multi-turn total rotation angle of the turret
        if (Math.abs(delta)<500) // ignore hysteresis
            currentTurretAngle = (servoTurnCount * 360 + servoAngle)*GEAR_RATIO;

        // Forward wrap detection (jumped from +180 → -180)
        if (delta < -4500) servoTurnCount++;

        // Reverse wrap detection (jumped from -180 → +180)
        if (delta > 4500) servoTurnCount--;

        turretHeadingControl.setPID(turretP, turretI, turretD);

        servoPower = turretHeadingControl.calculate(currentTurretAngle) + turretF*Math.signum(turretHeadingControl.getPositionError());
        //servo1.set(servoPower);
        //servo2.set(servoPower);
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

        telem.logDrivers("Heading Lock Mode", mode);
        telem.logDrivers("Turret Current Angle", currentTurretAngle);
        telem.logDrivers("Turret Target Angle ", turretHeadingControl.getSetPoint());
        telem.logDrivers("Turret Power", servoPower);
        telem.logDrivers("Turret Angle Error", turretHeadingControl.getPositionError());
        telem.logDrivers("Joystick Angle", joystickAngle);
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
            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                if (mode == HeadingLockMode.Off) {
                    mode = HeadingLockMode.Both;
                } else {
                    mode = HeadingLockMode.Off;
                }
            }
            if (mode == HeadingLockMode.Off)
                faceRobotAngle(0);
            if (mode == HeadingLockMode.Trig)
                faceFieldAngle(apriltag_heading);
            if (mode == HeadingLockMode.Camera) {
                aprilTagLock();
            }
            if (mode == HeadingLockMode.Both){
                if(! aprilTagLock()){
                    faceFieldAngle(apriltag_heading);
                }
            }

            /*
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
                //joystickAngle += 10;
                joystickAngle = 90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
               // joystickAngle -= 10;
                joystickAngle = -90;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                joystickAngle = 0;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                joystickAngle = 180;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                reset();
            }

            faceRobotAngle(joystickAngle);

             */
        }
    }

    private boolean aprilTagLock() {
        List<AprilTagDetection> detections = april_tags.getDetections();
        for (AprilTagDetection tag : detections) {
            if (tag.id == target.id){
                //drive.april_bearing = drive.getPosition().getHeading(AngleUnit.DEGREES) + tag.ftcPose.bearing;
                faceRobotAngle(tag.ftcPose.bearing + currentTurretAngle);
                //telem.log("bearing", tag.ftcPose.bearing);
                //range(distance)is in inches, maybe convert to centi
                //telem.log("distance to april tag, inches", tag.ftcPose.range);
                return true;
            }
        }
        return false;
    }
}
