package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

public class Turret extends SubsystemBase {
    HyperTelemetry telem;
    Servo turret;
    public PIDController turretHeadingControl;
    double servoPower;
    double turretAngle;
    public double angleAdjust;

    public Turret(HardwareMap hardwareMap, HyperTelemetry telem){
        this.telem = telem;
        turret = hardwareMap.get(Servo.class, "turret");// servo port 0 on expansion
    }

}
