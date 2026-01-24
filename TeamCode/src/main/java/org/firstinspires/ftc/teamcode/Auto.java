package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Consumer;
import java.util.function.Function;


@Autonomous(name="AutoTest", group="Opmode")
public class Auto extends RobotBaseOp {

    // TODO: declare this class abstract, make AutoBlueFar / etc
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }

    public Follower follower;

    private final Pose blueFarStart = new Pose(47.5 + 8.124, 8.0984, Math.toRadians(90));
    private final Pose blueNearStart = new Pose(19.5 + 8.124,120.5 + 8.0984, Math.toRadians(90));
    private final Pose blueFarShoot = new Pose(
            47.5 + 8.124 + 2.0, // 2 inches further towards Red from start
            8.0984 + 10.0,// 10 inches in front of start position
            Math.toRadians(110)
        );
    private final Pose blueFarPark = new Pose(
            47.5 + 8.124, // same as start offset
            8.0984 + 25.0,// 25 inches in front of start position
            Math.toRadians(90)
        );
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
    private final Pose wallFar = new Pose(8.124 + 3, 23.6 + 8.098, Math.toRadians(250));
    private final Pose wallClose = new Pose(8.124 + 3, 8.098, Math.toRadians(250));

    private final Pose openGate = new Pose(10.4 + 7.0, 70, Math.toRadians(90));

    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}
    public boolean isAuto() { return true; }

    private Command _lastCommandRun = null;

    public void init(){
        super.init();
        follower = Constants.createFollower(hardwareMap);
    }

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

    private Command farBluePathing() {
        follower.setStartingPose(blueFarStart);

        SequentialCommandGroup auto = new SequentialCommandGroup();

        // shoot preloads
        auto.addCommands(
            pathBetween(blueFarStart, blueFarShoot, 1.0),
            new AutoOuttake()
        );

        // collect and shoot audience spike mark
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart1, 1.0),
            new ScheduleCommand(new AutoIntake()),
            pathBetween(spikeStart1, spikeEnd1, 0.45),
            pathBetween(spikeEnd1, blueFarShoot, 1.0),
            new AutoOuttake()
        );

        // collect and shoot human-player preloads
        auto.addCommands(
            pathBetween(blueFarShoot, wallFar, 1.0),
            new ScheduleCommand(new AutoIntake()),
            pathBetween(wallFar, wallClose, 0.45),
            pathBetween(wallClose, blueFarShoot, 1.0),
            new AutoOuttake()
        );

/*
        // collect and shoot middle spike mark
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart2, 1.0),
            new ScheduleCommand(new AutoIntake()),
            pathBetween(spikeStart2, spikeEnd2, 0.45),
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
        auto.addCommands(
                pathBetween(blueNearStart, blueNearShoot, 1.0),
                new AutoOuttake()
        );

        // pick up and shoot far spike mark
        auto.addCommands(
                pathBetween(blueNearShoot, spikeStart3, 1.0),
                new ScheduleCommand(new AutoIntake()),
                pathBetween(spikeStart3, spikeEnd3, 0.45)
        );
        // open the gate after picking up spike 3
        auto.addCommands(
                pathBetween(spikeEnd3, openGate, 1.0)
        );
        // shooting spike three after opening gate
        auto.addCommands(
                pathBetween(openGate, blueNearShoot, 1.0),
                new AutoOuttake()
        );
        // pick up and shoot middle spike mark
        auto.addCommands(
                pathBetween(blueNearShoot, spikeStart2, 1.0),
                new ScheduleCommand(new AutoIntake()),
                pathBetween(spikeStart2, spikeEnd2, 0.45),
                pathBetween(spikeEnd2, blueNearShoot, 1.0),
                new AutoOuttake()
        );

        return auto;
    }

    @Override
    public void start() {
        super.start();

        CommandScheduler.getInstance().onCommandExecute(this::commandRunning);

       // Command cmds = farBluePathing();
        Command cmds = nearBluePathing();

        CommandScheduler.getInstance().schedule(cmds);

        // tell the Spindexer about its preloads
        spindexer.slots[0] = Spindexer.SlotContent.Green;
        spindexer.slots[1] = Spindexer.SlotContent.Purple;
        spindexer.slots[2] = Spindexer.SlotContent.Purple;
    }

    public void commandRunning(Command c) {
        _lastCommandRun = c;
    }

    @Override
    protected void addTelemetry(HyperTelemetry telem) {
        super.addTelemetry(telem);
        if (_lastCommandRun != null) {
            telem.log("auto-command-run", _lastCommandRun);
            _lastCommandRun = null;
        }
    }

    @Override
    public void loop(){
        follower.update();
        // the command-scheduler is run in our super-class
        super.loop();
    }
}
