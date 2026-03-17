package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.paths.Path;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="pathingTest", group="Opmode")
public class PathingTest extends RobotBaseOp {
    public Alliance getAlliance() { return Alliance.BLUE; }
    public StartZone getStartZone() { return StartZone.FAR; }
    public boolean isAuto(){
        return true;
    }
    protected void bindOperatorControls(){
    }
    protected void bindDriverControls(){
    }

    public Follower follower;
    double xOffset = 8.124;
    double yOffset = 8.0984;
    private final Pose blueFarStart = new Pose(70 - yOffset, xOffset, Math.toRadians(180));
    private final Pose positionOne = new Pose(24 + yOffset,45,Math.toRadians(180));
    private final Pose positionTwo = new Pose (120 - yOffset, 45, Math.toRadians(180));
    Pose lastPose = blueFarStart;
    public static double brakingStrength = 1.0;
    public static double brakingStart = 1.3;
    private Command _lastCommandRun = null;

    public FollowPathCommand pathBetween(Pose b, Pose e, double speed) {
        Pose begin = b;
        Pose end = e;
        Path p = new Path(new BezierLine(begin, end));
        if (begin.getHeading() != end.getHeading()) {
            p.setLinearHeadingInterpolation(begin.getHeading(), end.getHeading());
        } else {
            p.setConstantHeadingInterpolation(end.getHeading());
        }
        // via Brogan M Pratt, default is 1.0 .. lower numbers stop SOONER.
        // brakingStart default is 1
        p.setBrakingStart(brakingStart);
        p.setBrakingStrength(brakingStrength);
        return new FollowPathCommand(p, speed);
    }

    class FollowPathCommand extends CommandBase {
        Path path;
        double speed;
        double startTime;

        public FollowPathCommand(Path p, double s) {
            path = p;
            speed = s;
            addRequirements(drive);
        }
        public void initialize() {
            follower.followPath(path);
            follower.setMaxPower(speed);
            startTime = time;
        }
        public boolean isFinished() {
            double elapsed = time - startTime;
            return !follower.isBusy() || elapsed > 3;
        }
    }

    class Delay extends CommandBase {
        double seconds;
        double start;
        public Delay(double s) {
            seconds = s;
        }
        public void initialize(){
            start = time;
        }
        public boolean isFinished(){
            return (time - start) > seconds;
        }
    }

    @Override
    public void init(){
        super.init();
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        readControls();

        telemetry.update();
    }

    @Override
    public void start() {
        // we must run this _before_ the "pathing" options because
        // those set the start-position of the robot
        super.start();

        Pose start = blueFarStart;
        drive.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        start.getX(),
                        start.getY(),
                        AngleUnit.RADIANS,
                        start.getHeading()
                )
        );
        follower.setStartingPose(start);


        // "Tuning.Line" tuner does this .. really needed?
        follower.activateAllPIDFs();

        Command cmds = null;
    }

    @Override
    protected void addTelemetry(HyperTelemetry telem) {

        super.addTelemetry(telem);
        telem.logBoth("Pinpoint-status", drive.pinpoint.getDeviceStatus());
    }

    @Override
    public void loop(){
        super.loop();
        follower.update();

        if (driver.wasJustPressed(GamepadKeys.Button.X)){
            Command c = pathBetween(lastPose,positionOne, 1.0);
            CommandScheduler.getInstance().schedule(c);
            lastPose = positionOne;

        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)){
            Command c = pathBetween(lastPose, positionTwo, 1.0);
            CommandScheduler.getInstance().schedule(c);
            lastPose = positionTwo;
        }
        CommandScheduler.getInstance().run();

    }

    @Override
    public void stop() {

        super.stop();
    }
}
