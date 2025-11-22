package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {
    private CRServo servo1, servo2;
    AnalogInput encoder;

    public PIDController turretHeadingControl;
    private double targetAngle, currentAngle, lastAngle, servoPower;
    private int turnCount;
    double beta = 0, joystickAngle;
    double encoderOffset;
    double error = 0;

    private static final double GEAR_RATIO = 1/0.64; // last session Vincent, Avery and Mahie worked it out as 0.64:1 (i.e. 1 servo rotation equals 0.64 turret rotations)
    //public static double turretP = 0.013*GEAR_RATIO, turretI = 0.0, turretD = 0.0004*GEAR_RATIO, turretF = 0.015; // You MUST tune these
    public static double turretP = 0.005, turretI = 0.0, turretD = 0.0, turretF = 0.05; // You MUST tune these
    public static double TURRET_TOLERANCE = 2.0; // in degrees
//    private static FileWriter writer;

    public Turret(HardwareMap hardwareMap) {
        // both servos must always run in the same direction
        servo1 = new CRServo(hardwareMap, "left_turret"); servo1.setInverted(true);
        servo2 = new CRServo(hardwareMap, "right_turret"); servo2.setInverted(true);
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
        turnCount = 0;
        lastAngle = 0;
        currentAngle = 0;
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
        targetAngle = angle;
    }

    public void reset() {
        encoderOffset = rawEncoderAngle();
    }

    public double rawEncoderAngle() {
        return encoder.getVoltage()/3.3 * 360;
    }

    @Override
    public void periodic() {
        // Read analog voltage, convert to degrees
        double angle = rawEncoderAngle();
        // accoun for our "software reset" (the Axons in CR mode are absolute encoders)
        angle -= encoderOffset;
        // convert to single-turn angle (-180..180 deg)
        //currentAngle = wrapAngle(angle - 180);
        currentAngle = angle;

        /*
        // Wrap detection for multi-turn
        double delta = angle - lastAngle;

        // Multi-turn total angle
        if (Math.abs(delta)<10) // ignore hysteresis
            currentAngle = (turnCount * 360 + angle)/GEAR_RATIO;

        // Forward wrap (jumped from +180 → -180)
        if (delta < -175) turnCount++;

        // Reverse wrap (jumped from -180 → +180)
        if (delta > 175) turnCount--;

        lastAngle = angle;
        */

        turretHeadingControl.setPID(turretP, turretI, turretD);
        error = wrapAngle(targetAngle-currentAngle);
        //servoPower = turretHeadingControl.calculate(error) - (turretF * Math.signum(error));
        servo1.set(servoPower);
        servo2.set(servoPower);
//        try { writer.write(angle+" "+currentAngle+"\n"); } catch (IOException e) { e.printStackTrace(); }
    }

    private static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }

    public boolean isFinished() {
        // check if the target is reached
        return turretHeadingControl.atSetPoint();
    }
    public void stop() {
        servo1.stop();
        servo2.stop();
//        try { writer.close(); } catch (IOException e) { e.printStackTrace(); }
    }

    public void add_telemetry(TelemetryPacket pack, Telemetry telemetry) {
        pack.put("turret-current-angle", currentAngle);
        pack.put("turret-target-angle", targetAngle);
        pack.put("turret-power", servoPower);
        pack.put("turret-error", error);
        pack.put("turret-joystick", joystickAngle);
        pack.put("turret-beta", beta);
        pack.put("turret-offset", encoderOffset);

        telemetry.addData("Turret Current Angle", currentAngle);
        telemetry.addData("Turret Target Angle ", targetAngle);
        telemetry.addData("Turret Power", servoPower);
        telemetry.addData("Heading Offset", beta);
        telemetry.addData("Joystick Angle", joystickAngle);
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
            double rx = operator.getRightX();
            double ry = operator.getRightY();
            joystickAngle = Math.toDegrees(Math.atan2(ry, rx));
            double mag = Math.hypot(rx, ry);

            if (mag > 0.8) {
                joystickAngle = Math.atan2(rx, -ry) * 360 / 2 / 3.14159;
                beta = 0;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                beta += 10;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                beta -= 10;
            }
            // only do the joystick control if it has moved "a lot" (1.0 is slammed)
            if (Math.hypot(ry,rx)> 0.8) {
                beta = joystickAngle;
            }
            //faceRobotAngle(beta);
            servoPower = operator.getRightX();
        }
    }
}
