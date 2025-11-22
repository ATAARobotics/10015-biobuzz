package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {
    private CRServo servo1;
    private CRServo servo2;
    AnalogInput encoder;

    private PIDController turretHeadingControl;
    double servoPower;

    public static double turretP = 1.0, turretI = 0.0, turretD = 0.0;
    public static double TURRET_TOLERANCE = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "left_turret");
        servo2 = hardwareMap.get(CRServo.class, "right_turret");
        encoder = hardwareMap.get(AnalogInput.class, "left_encoder");
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
    }

    public void faceFieldAngle(double angle) {
        //faceRobotAngle(); //22.755 ticks/deg
    }
    public void faceRobotAngle(double angle) {
        turretHeadingControl.setSetPoint(angle);
        double turretPower = clipPower(turretHeadingControl.calculate(getCurrentAngle()) / 360); //degrees
        servoPower = (turretPower + 1.0) / 2.0;  // scale to 0.0 -> 1.0
        servo1.setPower(servoPower);
        servo2.setPower(servoPower);
    }
    public double getCurrentAngle() {
        return encoder.getVoltage()/3.3*360; // 0-360 deg
    }
    public double clipPower(double power) {
        if (power > 1) {return 1;}
        if (power < -1) {return -1;}
        return power;
    }
    public void add_telemetry(TelemetryPacket pack) {
        pack.put("turret-d", turretD);
        pack.put("turret-i", turretI);
        pack.put("turret-p", turretP);
        if (turretHeadingControl != null) {
            pack.put("turret-angle", turretHeadingControl.getSetPoint());
        }
        pack.put("turret-tolerance", TURRET_TOLERANCE);
        pack.put("servo-power", servoPower);
    }
}