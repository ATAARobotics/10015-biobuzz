package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {
    private CRServo servo1, servo2;
    AnalogInput encoder;

    public PIDFController turretHeadingControl;
    private double targetAngle, currentAngle, lastAngle, servoPower;
    private int turnCount;

    private static final double GEAR_RATIO = 1/0.64; // last session Vincent, Avery and Mahie worked it out as 0.64:1 (i.e. 1 servo rotation equals 0.64 turret rotations)
    //public static double turretP = 0.013*GEAR_RATIO, turretI = 0.0, turretD = 0.0004*GEAR_RATIO, turretF = 0.015; // You MUST tune these
    public static double turretP = 0.005, turretI = 0.0, turretD = 0.0001, turretF = 0.0; // You MUST tune these
    public static double TURRET_TOLERANCE = 2.0; // in degrees
//    private static FileWriter writer;

    public Turret(HardwareMap hardwareMap) {
        // both servos must always run in the same direction
        servo1 = new CRServo(hardwareMap, "left_turret"); servo1.setInverted(false);
        servo2 = new CRServo(hardwareMap, "right_turret"); servo2.setInverted(false);
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDFController(turretP,turretI,turretD,turretF);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
        turnCount = 0;
        lastAngle = 0;
        currentAngle = 0;
//        try { writer = new FileWriter("/sdcard/FIRST/axon_debug.txt"); } catch (IOException e) { e.printStackTrace(); }
    }
    public void faceFieldAngle(double angle) {
        //faceRobotAngle(); //22.755 ticks/ deg
    }

    public void faceRobotAngle(double angle) { targetAngle = angle;}

    @Override
    public void periodic() {
        // Read analog voltage and convert to single-turn angle (-180..180 deg)
        double angle = encoder.getVoltage()/3.3 * 360 - 180;

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

        turretHeadingControl.setPIDF(turretP, turretI, turretD, turretF);
        servoPower = turretHeadingControl.calculate(wrapAngle(targetAngle-currentAngle));
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
        pack.put("turret-target-angle", currentAngle);
        pack.put("turret-power", servoPower);
;
        telemetry.addData("Turret Current Angle", currentAngle);
        telemetry.addData("Turret Target Angle ", targetAngle);
        telemetry.addData("Turret Power", servoPower);
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

            // give turret target angles between -180 and 180
            faceRobotAngle(-operator.getRightX() * 180);
        }
    }
}
