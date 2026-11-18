package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Turret extends SubsystemBase {
    HyperTelemetry telem;

    CRServo turret;
    DcMotor mellonEncoder;
    
    public PIDController control;
    public static double turretP = 0.01; //turret tuned nov 16
    public static double turretI = 0.0001;
    public static double turretD = 0.0005;
    public static double turretF = 0.10;
    public static double ALIGNMENT_TOLERANCE = 4.0; // degrees

    public static double overridePower = 0.0;

    // fixme not static
    public static double targetAngle = -90;
    double servoPower;
    double turretAngle;
    public static final double GEAR_RATIO = 0.4231;
    private static final double TICKS_PER_REV =  4096;
    double error;
    double ticksMoved;
    double initialTicks;
    double encoderRevs;
    public static double START_ANGLE = -90;

    public Turret(HardwareMap hardwareMap, HyperTelemetry telem){
        this.telem = telem;
        turret = new CRServo(hardwareMap, "turret"); // servo port 0 on expansion
        control = new PIDController(turretP, turretI, turretD);
        mellonEncoder = hardwareMap.get(DcMotor.class, "intake");//reuse intake encoder wire. Port 0 expansion hub
    }

    public void start(){
        initialTicks = mellonEncoder.getCurrentPosition();
    }
    public boolean isReady() {
	return (error < ALIGNMENT_TOLERANCE);
    }
    public void faceRobotAngle(double angle) {
        double maxAngle = 160;
        if (angle < -maxAngle) angle = -maxAngle;
        if (angle > maxAngle) angle = maxAngle;
        targetAngle = angle;
        // "set point" is always zero and we compute the error ourselves
        control.setSetPoint(0);
    }

    @Override
    public void periodic(){
        control.setPID(turretP, turretI, turretD);
        ticksMoved = mellonEncoder.getCurrentPosition() - initialTicks;
        encoderRevs = ticksMoved / TICKS_PER_REV;
        turretAngle = START_ANGLE + encoderRevs * 360 * GEAR_RATIO;
        error = targetAngle - turretAngle;

        servoPower = control.calculate(error);

        // we just want a simple "static friction" style F and our
        // setpoint is always 0 so the solverslib one doesn't do the
        // right thing.
        if (Math.abs(servoPower) > 0.05) {
            servoPower += Math.signum(servoPower) * turretF;
        }
        
        telem.logBoth("turret-angle", turretAngle);
        telem.logBoth("turret-error", error);
        telem.logBoth("turret-power", servoPower);

        if (overridePower != 0) servoPower = overridePower;

        // clamp powers between -1 and +1
        if (servoPower > 1.0) servoPower = 1.0;
        if (servoPower < -1.0) servoPower = -1.0;

        // additionally: if we're trying to move "the wrong way" past
        // a soft limit, stop it
        // we use positive servo power to move towards NEGATIVE angles
	if (servoPower > 0.0 && turretAngle < -100) {
	    servoPower = 0.0;
	}
	if (servoPower < 0.0 && turretAngle > 100) {
	    servoPower = 0.0;
	}

        turret.set(servoPower);
    }

}
