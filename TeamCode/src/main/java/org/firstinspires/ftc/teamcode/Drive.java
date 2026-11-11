package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.controllers.Controller;

@Configurable
public class Drive extends SubsystemBase {
    public static double MAX_SPEED = 0.75;
    MecanumDrive drivebase;
    GoBildaPinpointDriver pinpoint;
    Follower follower;
    DrivePowers powers;

    public Pose2D currentPosition;

    boolean isRedAlliance;
    boolean isAuto;

    double strafe;
    double forward;
    double turn;
    double lateral;
    Controller headingLock;
    double targetAngle = 180;
    HyperTelemetry telem;

    double xOffset = 8.124; //from side edge to center of the robot. Nov 11 2026
    double yOffset = 8.649; //from top edge to center of the robot, not including crash bar. Nov 11 2026
    
    public Drive(HardwareMap hardwareMap, boolean isRedAlliance, boolean isAuto, HyperTelemetry telem) {
        this.isRedAlliance = isRedAlliance;
        this.isAuto = isAuto;
        this.telem = telem;

        Motor motor_fl = new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435); //port 0
        Motor motor_fr = new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_435); //port 1
        Motor motor_bl = new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_435); //port 2
        Motor motor_br = new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_435); //port 3
        motor_fr.setInverted(true);
        motor_br.setInverted(true);
        drivebase = new MecanumDrive(false, motor_fl, motor_fr, motor_bl, motor_br);

        // configure odometry sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Offsets from Nov 11 2026
        pinpoint.setOffsets(6.5131, 0.97, DistanceUnit.INCH); // Note: Y is forward, X is right.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD); //These have to match what is in Constants

        follower = Constants.create(hardwareMap);
        headingLock = Constants.foresightConfig.headingFeedback.get();

        pinpoint.resetPosAndIMU();
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
    }

    public void stop() {
        drivebase.stop();
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        currentPosition = pose;
        //desiredHeading = pose.getHeading(AngleUnit.DEGREES);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }
    public double bleuXToRedX(double x){
        x = -x + 141;
        return x;
    }
    public double blueYToRedY(double y){
        y = -y + 141;
        return y;
    }
    public double blueHeadingToRedHeading(double heading){
        heading += 180;
        heading %= 360;
        return heading;
    }
    public static double wrapAngle(double angleError) {
        angleError %= 360; // limit angle between -360 and +360
        if (angleError > 180)
            angleError -= 360;
        else if (angleError <= -180)
            angleError += 360;
        return angleError;
    }

    // all interaction with gamepads should go through this inner class
    public class HumanInputs extends CommandBase {
        GamepadEx driver;

        public HumanInputs(GamepadEx driver) {
            this.driver = driver;
            addRequirements(Drive.this);

        }

        @Override
        public void execute() {
            // Run wheels in POV mode: use the Right stick to go forward & strafe, the Left stick to rotate left & right.
            if (isRedAlliance) {
                strafe = driver.getRightX();
                forward = -driver.getRightY();
            } else {
                strafe = -driver.getRightX();
                forward = driver.getRightY();
            }

            turn = driver.getLeftX();

            // if we're holding left trigger _currently_, we go to
            // Turbo -- otherwise to non-Turbo
            ////turbo(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        }
    }

    // called ONCE, before any driver or robot inputs
    public void readSensors(double time) {
        // Get the latest pose, which includes the x and y coordinates, plus the heading angle
        pinpoint.update();
        currentPosition = pinpoint.getPosition();
    }

    @Override
    public void periodic() {
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                -strafe,
                -turn,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();
        drivebase.setMaxSpeed(MAX_SPEED);

        //drivebase.driveFieldCentric(strafe, forward, autoTurn, currentPosition.getHeading(AngleUnit.DEGREES), false);
    }
}
