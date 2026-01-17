package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="AutoTest", group="Opmode")
public class AutoTest extends OpMode {
   /* public Alliance getAlliance(){
        return Alliance.BLUE;
    }*/

    public Follower follower;
    private final Pose startPose = new Pose(56, 10, Math.toRadians(90)); // Start Pose of our robot.
   // private final Pose scorePose = new Pose(72, 72, Math.toRadians(0));
    private final Pose poseOne = new Pose (45,36, Math.toRadians(180));
    private final Pose poseTwo = new Pose (12.4,36.7, Math.toRadians(180));
    private Path pathOne;
    private Path pathTwo;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        pathOne = new Path(new BezierLine(startPose, poseOne));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), poseOne.getHeading());
        pathTwo = new Path(new BezierLine(poseOne, poseTwo));
        pathTwo.setLinearHeadingInterpolation(poseOne.getHeading(), poseTwo.getHeading());
    }
    public void start(){
        follower.followPath(pathOne);
        follower.followPath(pathTwo);
    }
    public void loop(){
        follower.update();
        telemetry.addData("busy", follower.isBusy());
    }
}
