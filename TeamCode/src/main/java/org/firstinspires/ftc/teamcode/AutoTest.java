package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
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

    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}

    class Operation {
    }

    class PathOperation extends Operation {
        public Path path;

        public PathOperation(Path p) {
            path = p;
        }
    }

    class CommandOperation extends Operation {
        public Command command;

        public CommandOperation(Command c) {
            command = c;
        }
    }

    public void init(){
        super.init();
        follower = Constants.createFollower(hardwareMap);
        operations = new LinkedList<Operation>();
    }

    private void farBluePathing() {
        Pose startPose = blueFar;
        follower.setStartingPose(startPose);

        Pose one = new Pose (45,36, Math.toRadians(180));
        Pose two = new Pose (12.4,36.7, Math.toRadians(180));
        Path pathOne = new Path(new BezierLine(startPose, one));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), one.getHeading());
        Path pathTwo = new Path(new BezierLine(one, two));
        pathTwo.setLinearHeadingInterpolation(one.getHeading(), two.getHeading());

        operations.addLast(new PathOperation(pathOne));
        operations.addLast(new CommandOperation(new AutoIntake()));
        operations.addLast(new PathOperation(pathTwo));
        // "cancel autointake" command?
        // TODO: go to shoot position
        // TODO: run AutoOuttake() ... until done? until 3 shots?
    }

    public void start() {
        farBluePathing();
        ///follower.followPath(paths.getFirst());
    }

    public void loop(){
        follower.update();
        if (!follower.isBusy()) {
            if (operations.size() > 0) {
                Operation oper = operations.removeFirst();
                if (oper.getClass() == PathOperation.class) {
                    currentPath = ((PathOperation)oper).path;
                    follower.followPath(currentPath);
                } else if (oper.getClass() == CommandOperation.class) {
                    lastCommand = ((CommandOperation)oper).command;
                    CommandScheduler.getInstance().schedule(lastCommand);
                }
                // TODO: probably want a like "wait for last command
                // to complete" sort of thing? (e.g. to wait for all
                // the shots to fire)
            }
        }
    }
}
