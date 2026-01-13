package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name="AutoTest", group="Opmode")
public class AutoTest extends OpMode {
   /* public Alliance getAlliance(){
        return Alliance.BLUE;
    }*/

    public Follower follower;
    private final Pose startPose = new Pose(56, 10, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 72, Math.toRadians(0));
    private Path scorePreload;
    public void init(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }
    public void start(){
        follower.followPath(scorePreload);
    }
    public void loop(){
        follower.update();
        telemetry.addData("busy", follower.isBusy());
    }
}
