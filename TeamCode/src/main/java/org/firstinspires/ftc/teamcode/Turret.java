package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {
    private SimpleServo servo1;
    private SimpleServo servo2;
    AnalogInput encoder;

    public PIDController turretHeadingControl;
    public double servoPower;

    public static double turretP = 0.0, turretI = 0.0, turretD = 0.0;
    public static double F = 0.0;
    public static double TURRET_TOLERANCE = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servo1 = new SimpleServo(hardwareMap, "left_turret", 0, 360);
        servo2 = new SimpleServo(hardwareMap, "right_turret", 0, 360);
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
    }

    public void faceFieldAngle(double angle) {
        //faceRobotAngle(); //22.755 ticks/ deg
    }
    public void faceRobotAngle(double angle) {
        turretHeadingControl.setPID(turretP, turretI, turretD);
        turretHeadingControl.setSetPoint(angle);
    }

    @Override
    public void periodic() {
        // our controller is in degrees; if this changes, P, I, D and F need to be re-tuned
        // raw PID controller power
        double turretPower = turretHeadingControl.calculate(getCurrentAngle());
        // add F and clip to between -1.0 and 1.0
        turretPower = clipPower(turretPower + F);
        // re-scale to the servo range
        servoPower = (turretPower + 1.0) / 2.0;  // scale to 0.0 -> 1.0
        servo1.setPosition(servoPower);
        servo2.setPosition(servoPower);
    }

    public double getCurrentAngle() {
        return encoder.getVoltage()/3.3*360; // 0-360 deg
    }

    public double clipPower(double power) {
        if (power > 1) {return 1;}
        if (power < -1) {return -1;}
        return power;
    }

    public void add_telemetry(TelemetryPacket pack, Telemetry telemetry) {
        pack.put("turret-d", turretD);
        pack.put("turret-i", turretI);
        pack.put("turret-p", turretP);
        if (turretHeadingControl != null) {
            pack.put("turret-angle", turretHeadingControl.getSetPoint());
        }
        pack.put("turret-tolerance", TURRET_TOLERANCE);
        pack.put("servo-power", servoPower);
;
        telemetry.addData("Turret Encoder Angle: ", getCurrentAngle());
        telemetry.addData("Turret Target Angle:  ", turretHeadingControl.getSetPoint());
        telemetry.addData("Turret Power: ", servoPower);
    }
}
