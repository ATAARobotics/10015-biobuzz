package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends SubsystemBase {
    private CRServo servo1;
    private DcMotor encoder1;
    private CRServo servo2;
    private DcMotor encoder2;

    private PIDController turretHeadingControl;

    public double turretP = 0.0, turretI = 0.0, turretD = 0.0;
    public double TURRET_TOLERANCE = 0.0;

    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "turretServo1");
        servo2 = hardwareMap.get(CRServo.class, "turretServo2");
        encoder1 = hardwareMap.get(DcMotor.class, "turretEncoder1");
        encoder2 = hardwareMap.get(DcMotor.class, "turretEncoder2");
    }

    public void faceFieldAngle(double angle) {
        //faceRobotAngle(); //22.755 ticks/ deg
    }
    public void faceRobotAngle(double angle) {
        turretHeadingControl = new PIDController(turretP,turretI,turretD);
        turretHeadingControl.setTolerance(TURRET_TOLERANCE);
        servo1.setPower((turretHeadingControl.calculate(encoder2.getCurrentPosition())+1)/2);
    }
    public void add_telemetry(TelemetryPacket pack) {
        pack.put("turret-d", turretD);
        pack.put("turret-i", turretI);
        pack.put("turret-p", turretP);
        pack.put("turret-tolerance", TURRET_TOLERANCE);
    }
}