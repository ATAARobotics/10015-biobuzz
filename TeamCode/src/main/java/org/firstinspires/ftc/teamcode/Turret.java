package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Turret extends SubsystemBase {
    HyperTelemetry telem;
    Drive drive;
    CRServo turret;
    DcMotor melonEncoder;
    
    public PIDController control;
    public static double turretP = 0.007; //turret tuned nov 27
    public static double turretI = 0.0;
    public static double turretD = 0.0002;
    public static double turretF = 0.115;
    public static double FRICTION_BOOST_THRESHOLD = 0.0001;
    public static double ALIGNMENT_TOLERANCE = 4.0; // degrees
    public static double MANUAL_DIVISOR = 15;
    public static double overridePower = 0.0;

    // fixme not static
    public static double targetHeading = -90;
    double servoPower;
    double turretAngle;
    public static final double GEAR_RATIO = 0.4231;
    private static final double TICKS_PER_REV =  4096;
    double error;
    double ticksMoved;
    double encoderRevs;
    public static double START_ANGLE = -90;
    double operatorOffset;
    double OPERATOR_INCREMENT = 1;
    public enum Mode {Auto, Manual}
    public Mode mode = Mode.Auto;
    double manualPower;
    public Turret(HardwareMap hardwareMap, HyperTelemetry telem, Drive drive, boolean isAuto){
        this.telem = telem;
        this.drive = drive;
        turret = new CRServo(hardwareMap, "turret"); // servo port 0 on expansion
        control = new PIDController(turretP, turretI, turretD);
        melonEncoder = hardwareMap.get(DcMotor.class, "intake");//reuse intake encoder wire. Port 0 expansion hub
        if(isAuto || !OpmodeStorage.autoRan){
            melonEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        melonEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void start(){
        mode = Mode.Auto;
    }
    public boolean isReady() {
	return (Math.abs(error) < ALIGNMENT_TOLERANCE);
    }
    public void faceRobotAngle(double angle) {
        double maxAngle = 90;
        if (angle < -maxAngle) angle = -maxAngle;
        if (angle > maxAngle) angle = maxAngle;
        targetHeading = angle;
        // "set point" is always zero and we compute the error ourselves
        control.setSetPoint(0);
    }
    public static double wrapAngle(double angleError) {
        angleError %= 360; // limit angle between -360 and +360
        if (angleError > 180)
            angleError -= 360;
        else if (angleError <= -180)
            angleError += 360;
        return angleError;
    }
    public void faceFieldAngle(double fieldAngleDeg) {
        double robotHeading = drive.getPosition().getHeading(AngleUnit.DEGREES);
        double robotRelative = fieldAngleDeg - robotHeading;
        //robotRelative = wrapAngle(robotRelative);

        // Reuse existing robot-relative method
        faceRobotAngle(robotRelative);
    }
    public void plusOperatorTweak(){
        operatorOffset += OPERATOR_INCREMENT;
    }
    public void minusOperatorTweak(){
        operatorOffset -= OPERATOR_INCREMENT;
    }
    public void reset() {
        turretAngle = START_ANGLE;
        targetHeading = START_ANGLE;
        melonEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        melonEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void periodic(){
        control.setPID(turretP, turretI, turretD);
        ticksMoved = melonEncoder.getCurrentPosition();
        encoderRevs = ticksMoved / TICKS_PER_REV;
        turretAngle = START_ANGLE + encoderRevs * 360 * GEAR_RATIO;
        error = targetHeading + operatorOffset - turretAngle;
        if(mode == Mode.Manual) {
            servoPower = manualPower;
        } else{
            servoPower = control.calculate(error);
        }
        // we just want a simple "static friction" style F and our
        // setpoint is always 0 so the solverslib one doesn't do the
        // right thing.
        if (Math.abs(servoPower) > FRICTION_BOOST_THRESHOLD) {
            servoPower += Math.signum(servoPower) * turretF;
        }
        
        telem.log("turret-angle", turretAngle);
        telem.log("turret-target", targetHeading);
        telem.log("turret-error", error);
        telem.log("turret-power", servoPower);
        telem.log("turret-operator-offset", operatorOffset);

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
    public class HumanInputs extends CommandBase {
        GamepadEx operator;
        public HumanInputs(GamepadEx operator) {
            this.operator = operator;
            addRequirements(Turret.this);
        }

        @Override
        public void execute() {
            manualPower = operator.getLeftX() / MANUAL_DIVISOR;
            if (operator.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                mode = Mode.Manual;
            }
            else{
                if (mode == Mode.Manual){
                    mode = Mode.Auto;
                    reset();
                }
            }
        }
    }
}
