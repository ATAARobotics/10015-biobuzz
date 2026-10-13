package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
@Autonomous
public class PollenPathing extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    double xOffset = 7.179;
    double yOffset = 7.19;

    private final Pose startPose = p.of(23.5, yOffset, 90);
    private final Pose pointOne = p.of(23.5,117.5,90);
    private Path forward() {
        return line(startPose, pointOne).linear(startPose, pointOne);
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

    }
    @Override
    public void start() {
        schedule(follow(follower, forward()));
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}