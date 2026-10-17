package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
public abstract class Auto extends RobotBaseOp {
    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}
    public boolean isAuto() { return true; }

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    double xOffset = 7.179;
    double yOffset = 7.19;

    private final Pose startNear = p.of(47 + xOffset, yOffset, 90);//Starting Pose for side where hive is tipped up
    private final Pose garden = p.of(10.0,9.14,260);//Pose to prepare to intake garden
    private final Pose controlGarden = p.of(35.3, 36.9, 90); //Control point on way to pick up garden
    private final Pose shootFar = p.of(58,118,270);//Shoot at the far side
    private final Pose shootFarControlPoint = p.of(32.1,135.8,270); // control point when driving from garden to far shoot
    private final Pose pointTowardsFlower = p.of(58,119,90); //rotate to point towards flower
    private final Pose farFlowerPickUp = p.of(46.4,126,90);//Pick up the flower at the far side
    private final Pose endParkSolo = p.of(8.64,115,0); //Parking in zone at end alone


    private Path gardenPickUp() {
        return curve(startNear, controlGarden, garden).linear(startNear, garden);
    }
    private Path goToFarShoot() {
        return curve(garden, shootFarControlPoint, shootFar).linear(garden, shootFar);
    }
    private Path turnToFlower(){
        return line(shootFar, pointTowardsFlower).linear(shootFar,pointTowardsFlower);
    }
    private Path pickUpFarFlower() {
        return line(shootFar, farFlowerPickUp).linear(shootFar, farFlowerPickUp);
    }
    private Path shootFromFarFlower() {
        return line(farFlowerPickUp, shootFar).linear(farFlowerPickUp, shootFar);
    }
    private Path parkFromFarShoot() {
        return line(shootFar, endParkSolo).linear(shootFar, endParkSolo);
    }

    private Command autoRoutineNear() {
        return sequential(
                //Shoot preloads first from starting position
                follow(follower, gardenPickUp()),
                //run intake
                follow(follower, goToFarShoot()), // Add partner bump option
                //Shoot
                follow(follower, turnToFlower()),
                follow(follower, pickUpFarFlower()),
                //intake flower
                follow(follower, shootFromFarFlower()),
                //shoot
                follow(follower, parkFromFarShoot())
        );
    }
    private Command autoRoutineFar(){
        return sequential(
            //far pathing
        );

    }
    @Override
    public void init() {
        super.init();
        // ADD IN START FAR THING IF STATEMENT ONCE WRITTEN!!!!!!!!!!
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startNear);//start far if selected

    }
    @Override
    public void start() {
        schedule(autoRoutineNear());
    }//Make far an option here
    @Override
    public void loop() {
        super.loop();
        follower.update();
        Scheduler.execute();
    }
}
