package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Consumer;
import java.util.function.Function;

import kotlinx.coroutines.Delay;


public abstract class Auto extends RobotBaseOp {

    public enum StartZone {NEAR, FAR}

    public abstract Alliance getAlliance();
    public abstract StartZone getStartZone();

    public Follower follower;
    double xOffset = 8.124;
    double yOffset = 8.0984;
    private boolean usePreloads = false;
    private int whenOpenGate = 0;

    private final Pose blueFarStart = new Pose(47.5 + xOffset, yOffset, Math.toRadians(90));
    private final Pose blueNearStart = new Pose(19.5 + xOffset,120.5 + yOffset, Math.toRadians(90));
    private final Pose blueFarShoot = new Pose(
            47.5 + 8.124 + 2.0, // 2 inches further towards Red from start
            yOffset + 10.0,// 10 inches in front of start position
            Math.toRadians(110)
        );
    private final Pose blueFarPark = new Pose(
            47.5 + xOffset, // same as start offset
            yOffset + 25.0,// 25 inches in front of start position
            Math.toRadians(90)
        );
    private final Pose blueNearPark = new Pose(23.5 + xOffset, 23.5*3, Math.toRadians(270));
    private final Pose blueNearShoot = new Pose(
            45, // stay inside our side of the field
            105.0, // right at top of the cone
            Math.toRadians(180)
        );

    private final Pose spikeStart1 = new Pose (50, 35.0, Math.toRadians(180));
    private final Pose spikeEnd1 = new Pose (10.4, 35.0, Math.toRadians(180));

    // next set of spikes is one tile away
    private final Pose spikeStart2 = new Pose (50, 35.0 + 23.5, Math.toRadians(180));
    private final Pose spikeEnd2 = new Pose (10.4, 35.0 + 23.5, Math.toRadians(180));

    // closest set of spikes has the ramp in the way so we can't drive as far forward
    private final Pose spikeStart3 = new Pose (50, 35.0 + (2 * 23.5), Math.toRadians(180));
    private final Pose spikeEnd3 = new Pose (10.4 + 7.0, 35.0 + (2 * 23.5), Math.toRadians(180));

    // human-player preloads
    private final Pose wallFar = new Pose(xOffset + 3, 23.6 + yOffset, Math.toRadians(250));
    private final Pose wallClose = new Pose(xOffset + 3, yOffset + 3, Math.toRadians(250));

    // trying a different human-player routing
    private final Pose wallDirectStart = new Pose(xOffset + 1, yOffset + 15, Math.toRadians(240));
    private final Pose wallDirectEnd = new Pose(xOffset + 1, yOffset, Math.toRadians(240));

    private final Pose openGate = new Pose(10.4 + 7.0, 70, Math.toRadians(90));

    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}
    public boolean isAuto() { return true; }

    private Command _lastCommandRun = null;

    public Command pathBetween(Pose begin, Pose end, double speed) {
        PathChain p = new PathBuilder(follower)
            .addPath(new BezierLine(begin, end))
            .setLinearHeadingInterpolation(begin.getHeading(), end.getHeading())
            .build();

        return new FollowPathCommand(p, speed);
    }

    class FollowPathCommand extends CommandBase {
        PathChain path;
        double speed;

        public FollowPathCommand(PathChain p, double s) {
            path = p;
            speed = s;
            addRequirements(drive);
        }
        public void initialize() {
            follower.followPath(path);
            follower.setMaxPower(speed);
        }
        public boolean isFinished() {
            return !follower.isBusy();
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


    private Command farBluePathing() {
        drive.setPosition(
            new Pose2D(
                DistanceUnit.INCH,
                blueFarStart.getX(),
                blueFarStart.getY(),
                AngleUnit.RADIANS,
                blueFarStart.getHeading()
            )
        );
        follower.setStartingPose(blueFarStart);

        SequentialCommandGroup auto = new SequentialCommandGroup();

        auto.addCommands(
            pathBetween(blueFarStart, blueFarShoot, 1.0)
        );
        if (usePreloads) {
            auto.addCommands(new AutoOuttake());
        }

/*
        PathChain wallpickup = new PathBuilder(follower)
            .addPath(new BezierLine(blueFarShoot, wallDirectStart))
            .addPath(new BezierLine(wallDirectStart, wallDirectEnd))
            .setConstantHeadingInterpolation(wallDirectStart.getHeading())
            .build();

        auto.addCommands(
            new ParallelRaceGroup(
                new AutoIntake(),
                new FollowPathCommand(wallpickup, 0.9)
            ),
            pathBetween(wallDirectEnd, blueFarShoot, 1.0),
            new AutoOuttake()
        );
*/

        // collect and shoot audience spike mark
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart1, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                pathBetween(spikeStart1, spikeEnd1, 0.45)
            ),
            pathBetween(spikeEnd1, blueFarShoot, 1.0),
            new AutoOuttake()
        );

        // collect and shoot human-player preloads
        auto.addCommands(
            pathBetween(blueFarShoot, wallFar, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                new SequentialCommandGroup(
                        pathBetween(wallFar, wallClose, 0.55),
                        new Delay(1.0)
                )
            ),
            pathBetween(wallClose, blueFarShoot, 1.0),
            new AutoOuttake()
        );

/*
        // collect and shoot middle spike mark
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart2, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                pathBetween(spikeStart2, spikeEnd2, 0.45)
            ),
            pathBetween(spikeEnd2, blueFarShoot, 1.0),
            new AutoOuttake()
        );
*/

        // park off the start lines
        auto.addCommands(
            pathBetween(blueFarShoot, blueFarPark, 1.0)
        );

        return auto;
    }
    private Command nearBluePathing() {
        follower.setStartingPose(blueNearStart);

        SequentialCommandGroup auto = new SequentialCommandGroup();

        // shoot preloads
        Pose spikeStart = blueNearShoot;
        if (usePreloads) {
            auto.addCommands(
                pathBetween(blueNearStart, blueNearShoot, 1.0),
                new AutoOuttake()
                );
        } else {
            spikeStart = blueNearStart;
        }

        // pick up and shoot far spike mark (note our start position
        // depends on whether usePreloads was active or not)
        auto.addCommands(
                pathBetween(spikeStart, spikeStart3, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart3, spikeEnd3, 0.35)
                )
        );
       Pose lastSpike = spikeEnd3;
        if (whenOpenGate == 1) {
            // open the gate after picking up spike 3
            auto.addCommands(
                    pathBetween(spikeEnd3, openGate, 1.0)
            );
            lastSpike = openGate;
        }
        // shooting spike three after opening gate
        auto.addCommands(
                pathBetween(lastSpike, blueNearShoot, 1.0),
                new AutoOuttake()
        );
        // pick up and shoot middle spike mark
        auto.addCommands(
                pathBetween(blueNearShoot, spikeStart2, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart2, spikeEnd2, 0.35)
                ),
                pathBetween(spikeEnd2, spikeStart2, 1.0),
                pathBetween(spikeStart2, blueNearShoot, 1.0),
                new AutoOuttake()
        );

        return auto;
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

        if (operator.wasJustPressed(GamepadKeys.Button.A)) {
            usePreloads = !usePreloads;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.B)) {
            whenOpenGate += 1;
            if (whenOpenGate > 1) {
                whenOpenGate = 0;
            }
        }

        String openDescription = "unknown";
        if (whenOpenGate == 0) openDescription = "Never";
        if (whenOpenGate == 1) openDescription = "After Spike3 pickup";

        telemetry.addData("Preloads (A to toggle)", usePreloads);
        telemetry.addData("Open Gate (B to toggle)", openDescription);
        telemetry.update();
    }

    @Override
    public void start() {
        // we must run this _before_ the "pathing" options because
        // those set the start-position of the robot
        super.start();

        Command cmds = null;
        switch (getStartZone()) {
            case NEAR:
                cmds = nearBluePathing();
                break;
            case FAR:
                cmds = farBluePathing();
                break;
        }

        CommandScheduler.getInstance().schedule(cmds);

        // tell the Spindexer about its preloads
        if (usePreloads) {
            spindexer.slots[0] = Spindexer.SlotContent.Green;
            spindexer.slots[1] = Spindexer.SlotContent.Purple;
            spindexer.slots[2] = Spindexer.SlotContent.Purple;
        }
    }

    @Override
    protected void addTelemetry(HyperTelemetry telem) {
        super.addTelemetry(telem);
    }

    @Override
    public void loop(){
        follower.update();
        // the command-scheduler is run in our super-class
        super.loop();
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.INCH));
        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.INCH));
        editor.putFloat("turret", (float)turret.getServoAngle());
        editor.putFloat("spindex", (float)spindexer.targetAngle);
        editor.apply();
        drive.stop();
        shooter.stop();
    }
}
