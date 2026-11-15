package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Turret extends SubsystemBase {
    HyperTelemetry telem;
    Servo turret;
    DcMotor revEncoder;
    public PIDController turretHeadingControl;
    public static double turretP = 0;
    public static double turretI = 0;
    public static double turretD = 0;
    public double targetHeading;
    double servoPower;
    double turretAngle;
    public static final double GEAR_RATIO = 0.4231;
    private static final double REV_ENCODER_TICKS_PER_REV =  4096;
    double error;
    double ticksMoved;
    double encoderRevs;
    public static double START_ANGLE = -90;
    double startTicks = 0;
    public Turret(HardwareMap hardwareMap, HyperTelemetry telem){
        this.telem = telem;
        turret = hardwareMap.get(Servo.class, "turret"); // servo port 0 on expansion
        turretHeadingControl = new PIDController(turretP, turretI, turretD);
        revEncoder = hardwareMap.get(DcMotor.class, "intake");//reuse intake encoder wire. Port 0 expansion hub
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
        ticksMoved = revEncoder.getCurrentPosition() - startTicks;
        encoderRevs = ticksMoved/REV_ENCODER_TICKS_PER_REV;
        turretAngle = START_ANGLE + encoderRevs * 360 * GEAR_RATIO;
        turretAngle %= 360;

        servoPower = turretHeadingControl.calculate(error);
        telem.logBoth("turret-angle", turretAngle);

        if (servoPower > 1.0) servoPower = 1.0;
        if (servoPower < -1.0) servoPower = -1.0;
        //turret.set(servoPower);
    }

}
