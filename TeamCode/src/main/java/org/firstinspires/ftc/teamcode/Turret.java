package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {
    private final CRServo servo1, servo2;
    AnalogInput encoder;

    public PIDController turretHeadingControl;
    private double lastServoAngle, servoPower, currentTurretAngle;
    private int servoTurnCount;
    double joystickAngle;

    private static final double GEAR_RATIO = 0.8; // 1 servo rotation equals 0.8 turret rotations
    public static double turretP = 0.0035, turretI = 0, turretD = 0, turretF = 0.05; // Tuned 2025.11.23 with goBILDA 6V Servo Power Injector
    public static double TURRET_TOLERANCE = 1.0; // in degrees
//    private static FileWriter writer;

    public Turret(HardwareMap hardwareMap) {
        // both servos must always run in the same direction
        servo1 = new CRServo(hardwareMap, "left_turret");
        servo2 = new CRServo(hardwareMap, "right_turret");
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
        reset();
//        try { writer = new FileWriter("/sdcard/FIRST/axon_debug.txt"); } catch (IOException e) { e.printStackTrace(); }
    }

    public void faceFieldAngle(double fieldAngleDeg) {
        // Default if odometry isn't ready yet
        double robotHeadingDeg = 0.0;

        if (Drive.current_position != null) {
            robotHeadingDeg = Drive.current_position.getHeading(Drive.ANGLE_UNIT);
        }

        // Robot-relative angle: where turret must point relative to robot frame
        double robotRelative = wrapAngle(fieldAngleDeg - robotHeadingDeg);

        // Reuse existing robot-relative method
        faceRobotAngle(robotRelative);
    }

    public void faceRobotAngle(double angle) {
        turretHeadingControl.setSetPoint(angle);
    }

    public void reset() {
        currentTurretAngle = 0;
        lastServoAngle = 0;
        servoTurnCount = 0;
        faceRobotAngle(0);
        stop();
    }

    public double getServoAngle() {
        // Read analog voltage, convert to degrees
        return encoder.getVoltage()/3.3 * 360;
    }

    @Override
    public void periodic() {
        double servoAngle = getServoAngle();
        double delta = servoAngle - lastServoAngle;
        lastServoAngle = servoAngle;

        // Multi-turn total rotation angle of the turret
        if (Math.abs(delta)<10) // ignore hysteresis
            currentTurretAngle = (servoTurnCount * 360 + servoAngle)*GEAR_RATIO;

        // Forward wrap detection (jumped from +180 → -180)
        if (delta < -175) servoTurnCount++;

        // Reverse wrap detection (jumped from -180 → +180)
        if (delta > 175) servoTurnCount--;

        turretHeadingControl.setPID(turretP, turretI, turretD);

        servoPower = turretHeadingControl.calculate(currentTurretAngle) + turretF*Math.signum(turretHeadingControl.getPositionError());
        servo1.set(servoPower);
        servo2.set(servoPower);
//        try { writer.write(servoAngle+" "+currentTurretAngle+"\n"); } catch (IOException e) { e.printStackTrace(); }
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
                joystickAngle += 10;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                joystickAngle -= 10;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.Y)) {
                reset();
            }

            faceRobotAngle(joystickAngle);
        }
    }
}
