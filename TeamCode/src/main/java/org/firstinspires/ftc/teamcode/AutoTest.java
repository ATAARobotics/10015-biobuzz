package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.LinkedList;


@Autonomous(name="AutoTest", group="Opmode")
public class AutoTest extends OpMode {
   /* public Alliance getAlliance(){
        return Alliance.BLUE;
    }*/

    public Follower follower;
    private final Pose blueFar = new Pose(47.5 + 8.124, 8.0984, Math.toRadians(90));

    // we are "currently" following the first path
    LinkedList<Path> paths;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        paths = new LinkedList<Path>();
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

        paths.addLast(pathOne);
        paths.addLast(pathTwo);
    }
    public void start(){
        farBluePathing();
        follower.followPath(paths.getFirst());
    }
    public void loop(){
        follower.update();
        if (!follower.isBusy()) {
            paths.removeFirst();
            if (paths.size() > 0) {
                follower.followPath(paths.getFirst());
            }
        }
    }
}
