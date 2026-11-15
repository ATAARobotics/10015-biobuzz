package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;

public class Turret extends SubsystemBase {
    HyperTelemetry telem;
    CRServo turret;
    AnalogInput encoder;
    public PIDController turretHeadingControl;
    public static double turretP = 0;
    public static double turretI = 0;
    public static double turretD = 0;
    private static double TURRET_TOLERANCE;
    public double targetHeading;
    double servoPower;
    double turretAngle;
    public double robotHeading;
    public double angleAdjust;
    public static final double GEAR_RATIO = 0.4231;
    private static final double REV_ENCODER_TICKS_PER_REV =  4096;
    double error;
    double position;

    public Turret(HardwareMap hardwareMap, HyperTelemetry telem){
        this.telem = telem;
        turret = new CRServo(hardwareMap, "turret"); // servo port 0 on expansion
        encoder = hardwareMap.get(AnalogInput.class, "turret_encoder");

        turretHeadingControl = new PIDController(turretP, turretI, turretD);

    }

    public boolean isReady() {
        // TODO: Check turret alignment.
        return true;
    }
    public void faceRobotAngle(double angle) {
        double maxAngle = 160;
        if (angle < -maxAngle) angle = -maxAngle;
        if (angle > maxAngle) angle = maxAngle;
        targetHeading = angle;
        // "set point" is always zero and we compute the error ourselves
        turretHeadingControl.setSetPoint(0);
    }

    @Override
    public void periodic(){
        position = (encoder.getVoltage() / 3.2 * 360)/GEAR_RATIO;

        servoPower = turretHeadingControl.calculate(error);
        telem.logBoth("turret-angle", turretAngle);

        if (servoPower > 1.0) servoPower = 1.0;
        if (servoPower < -1.0) servoPower = -1.0;
        //turret.set(servoPower);
    }

}
