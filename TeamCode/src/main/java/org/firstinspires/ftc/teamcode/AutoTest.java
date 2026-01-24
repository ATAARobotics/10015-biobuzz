package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;


@Autonomous(name="AutoTest", group="Opmode")
public class AutoTest extends RobotBaseOp {

    // TODO: declare this class abstract, make AutoBlueFar / etc
    public Alliance getAlliance() {
        return Alliance.BLUE;
    }

    public Follower follower;
    private final Pose blueFar = new Pose(47.5 + 8.124, 8.0984, Math.toRadians(90));

    LinkedList<Operation> operations;
    Path currentPath = null;
    Command lastCommand = null;
    boolean waiting = false;

    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}

    class Operation {
    }

    class PathOperation extends Operation {
        public Path path;
        public double maxSpeed;

        public PathOperation(Path p, double m) {
            path = p;
            maxSpeed = m;  // 0.0 to 1.0
        }
    }

    class CommandOperation extends Operation {
        public Command command;

        public CommandOperation(Command c) {
            command = c;
        }
    }

    class WaitLastCommand extends Operation {
    }

    public void init(){
        super.init();
        follower = Constants.createFollower(hardwareMap);
        operations = new LinkedList<Operation>();
    }

    private void farBluePathing() {
        Pose startPose = blueFar;
        follower.setStartingPose(startPose);

        Pose blueFarShoot = new Pose(
            47.5 + 8.124 + 2.0, // 2 inches further towards Red from start
            8.0984 + 10.0,// 10 inches in front of start position
            Math.toRadians(110)
        );
        Pose blueFarPark = new Pose(
            47.5 + 8.124, // same as start offset
            8.0984 + 25.0,// 25 inches in front of start position
            Math.toRadians(90)
        );
        Pose blueCloseShoot = new Pose(
            72.0 - 11, // stay inside our side of the field
            72.0, // right at top of the cone
            Math.toRadians(180)
        );

        Pose spikeStart1 = new Pose (50, 35.0, Math.toRadians(180));
        Pose spikeEnd1 = new Pose (10.4, 35.0, Math.toRadians(180));

        // next set of spikes is one tile away
        Pose spikeStart2 = new Pose (50, 35.0 + 23.5, Math.toRadians(180));
        Pose spikeEnd2 = new Pose (10.4, 35.0 + 23.5, Math.toRadians(180));

        // closest set of spikes has the ramp in the way so we can't drive as far forward
        Pose spikeStart3 = new Pose (50, 35.0 + (2 * 23), Math.toRadians(180));
        Pose spikeEnd3 = new Pose (10.4 + 7.0, 35.0 + (2 * 23), Math.toRadians(180));

        Path pathZero = new Path(new BezierLine(startPose, blueFarShoot));
        pathZero.setConstantHeadingInterpolation(blueFarShoot.getHeading());

        Path pathOne = new Path(new BezierLine(blueFarShoot, spikeStart1));
        pathOne.setConstantHeadingInterpolation(spikeStart1.getHeading());
        Path pathTwo = new Path(new BezierLine(spikeStart1, spikeEnd1));
        pathTwo.setConstantHeadingInterpolation(spikeEnd1.getHeading());
        Path pathThree = new Path(new BezierLine(spikeEnd1, blueFarShoot));
        //Path pathThree = new Path(new BezierLine(spikeEnd1, blueCloseShoot));
        pathThree.setConstantHeadingInterpolation(blueFarShoot.getHeading());

        Path pathFour = new Path(new BezierLine(blueFarShoot, spikeStart2));
        pathFour.setConstantHeadingInterpolation(spikeStart2.getHeading());
        Path pathFive = new Path(new BezierLine(spikeStart2, spikeEnd2));
        pathFive.setConstantHeadingInterpolation(spikeEnd2.getHeading());
        Path pathSix = new Path(new BezierLine(spikeEnd2, blueFarShoot));
        pathSix.setConstantHeadingInterpolation(blueFarShoot.getHeading());

        Path pathSeven = new Path(new BezierLine(blueFarShoot, blueFarPark));
        pathSeven.setConstantHeadingInterpolation(blueFarPark.getHeading());


        // shoot preloads
        operations.addLast(new PathOperation(pathZero, 1.0));
        operations.addLast(new CommandOperation(new AutoOuttake()));
        operations.addLast(new WaitLastCommand());

        // furthest spike mark
        operations.addLast(new PathOperation(pathOne, 1.0));
        operations.addLast(new CommandOperation(new AutoIntake()));
        operations.addLast(new PathOperation(pathTwo, 0.5));
        operations.addLast(new PathOperation(pathThree, 1.0));
        operations.addLast(new CommandOperation(new AutoOuttake()));
        operations.addLast(new WaitLastCommand());

        // middle spike mark
        operations.addLast(new PathOperation(pathFour, 1.0));
        operations.addLast(new CommandOperation(new AutoIntake()));
        operations.addLast(new PathOperation(pathFive, 0.5));
        operations.addLast(new PathOperation(pathSix, 1.0));
        operations.addLast(new CommandOperation(new AutoOuttake()));
        operations.addLast(new WaitLastCommand());

        // park away from start lines
        operations.addLast(new PathOperation(pathSeven, 1.0));
    }

    @Override
    public void start() {
        super.start();
        farBluePathing();

        // tell the Spindexer about its preloads
        spindexer.slots[0] = Spindexer.SlotContent.Green;
        spindexer.slots[1] = Spindexer.SlotContent.Purple;
        spindexer.slots[2] = Spindexer.SlotContent.Purple;
    }

    @Override
    public void loop(){
        follower.update();
        if (waiting) {
            if (lastCommand == null) {
                waiting = false;
            } else {
                if (lastCommand.isFinished()) {
                    waiting = false;
                }
            }
        } else if (!follower.isBusy()) {
            if (operations.size() > 0) {
                Operation oper = operations.removeFirst();
                if (oper.getClass() == PathOperation.class) {
                    PathOperation po = (PathOperation)oper;
                    currentPath = po.path;
                    follower.followPath(currentPath);
                    follower.setMaxPower(po.maxSpeed);
                } else if (oper.getClass() == CommandOperation.class) {
                    lastCommand = ((CommandOperation)oper).command;
                    CommandScheduler.getInstance().schedule(lastCommand);
                } else if (oper.getClass() == WaitLastCommand.class) {
                    waiting = true;
                }
                // TODO: probably want a like "wait for last command
                // to complete" sort of thing? (e.g. to wait for all
                // the shots to fire when we get there?)
            }
        }
        super.loop();
    }
}
