package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitMs;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
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
    boolean noFarSide = false;
    boolean pushToPark = false;

    private final Pose startNear = p.of(47 + xOffset, yOffset, 90);//Starting Pose for side where hive is tipped up
    private final Pose garden = p.of(10.0,9.14,260);//Pose to prepare to intake garden
    private final Pose controlGarden = p.of(35.3, 36.9, 90); //Control point on way to pick up garden
    private final Pose shootFar = p.of(58,118,270);//Shoot at the far side
    private final Pose shootFarControlPoint = p.of(32.1,135.8,270); // control point when driving from garden to far shoot
    private final Pose pointTowardsFlower = p.of(58,118.1,90); //rotate to point towards flower
    private final Pose farFlowerPickUp = p.of(46.4,122,90);//Pick up the flower at the far side
    private final Pose endParkSolo = p.of(9.64,115,0); //Parking in zone at end alone
    private final Pose controlParkFromGarden = p.of(41.09, 57.37, 90); //Control point for option where we don't go to other side
    private final Pose parkOnEdge = p.of(8.25, 96.59,90); //Park closer to the edge to stay out of partner's way
    private final Pose prepareToBumpPartner = p.of(10.25,69.16,180); //So we don't destroy them
    private final Pose controlPointToPrepareToBumpPartner = p.of(41.09,57.37,90); //Exactly what it sounds like
    private final Pose bumpPartner = p.of(10.25,96.33,180); //Push partner into park position

    //far auto poses
    private final Pose startFar = p.of(72 - xOffset, 141 - yOffset, 270);
    private final Pose checkAprilTag = p.of(58.13, 108.1, 270);
    private final Pose generalShootingPositionFar = p.of(60.96, 132.4, 270);
    private final Pose controlPointSharedBetweenShootAndFlower = p.of(52, 85, 270);
    private final Pose flowerIntakeFar = p.of(47.13, 126, 90);
    private final Pose parkFar = p.of(9.44, 115.3, 270);
    private final Pose controlPointParkFar = p.of(63.5, 99.35, 270);

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
    private Path parkFromGarden(){
        return curve(garden, controlParkFromGarden, parkOnEdge).linear(garden,parkOnEdge);
    }
    private Path prepareToPush(){
        return curve(garden, controlPointToPrepareToBumpPartner, prepareToBumpPartner).linear(garden, prepareToBumpPartner);
    }
    private Path pushPartner(){
        return line(prepareToBumpPartner, bumpPartner).linear(prepareToBumpPartner, bumpPartner);
    }
    private Path bumpToShoot() {
        return line(bumpPartner, shootFar).linear(bumpPartner, shootFar);
    }
    //basic far auto paths
    private Path startToAprilTags(){
        return line(startFar, checkAprilTag).linear(startFar, checkAprilTag);
    }
    private Path aprilTagToShoot(){
        return line(checkAprilTag, generalShootingPositionFar).linear(checkAprilTag, generalShootingPositionFar);
    }
    private Path shootToFlowerIntake(){
        return curve(generalShootingPositionFar,controlPointSharedBetweenShootAndFlower, flowerIntakeFar).linear(generalShootingPositionFar, flowerIntakeFar);
    }
    private Path flowerToShoot(){
        return curve(flowerIntakeFar, controlPointSharedBetweenShootAndFlower, generalShootingPositionFar).linear(flowerIntakeFar, generalShootingPositionFar);
    }
    private Path shootToPark(){
        return curve(generalShootingPositionFar, controlPointParkFar, parkFar).linear(generalShootingPositionFar, parkFar);
    }

    private Command autoRoutineNear() {
	SequentialCommandGroup auto = new SequentialCommandGroup();
	//Shoot preloads first from starting position
    auto.addCommands(turnLightGreen());
	auto.addCommands(new FollowPathCommand(follower, gardenPickUp()));
	//run intake
    auto.addCommands(turnLightRed());
    if(noFarSide){
        auto.addCommands(new FollowPathCommand(follower, parkFromGarden()));
    }
    else {
        if(pushToPark){
            auto.addCommands(new FollowPathCommand(follower,prepareToPush()));
            //add delay here?
            auto.addCommands(new FollowPathCommand(follower,pushPartner()));
            auto.addCommands(new FollowPathCommand(follower,bumpToShoot())); //Going from parking to far shoot
        }else {
            auto.addCommands(new FollowPathCommand(follower, goToFarShoot())); //Going from garden to far shoot
            new Delay(1);
        }
        //Shoot
        auto.addCommands(turnLightGreen());
        auto.addCommands(new FollowPathCommand(follower, turnToFlower()));
        auto.addCommands(new FollowPathCommand(follower, pickUpFarFlower()));
        new Delay(1);
        //intake flower
        auto.addCommands(turnLightRed());
        auto.addCommands(new FollowPathCommand(follower, shootFromFarFlower()));
        //shoot
        auto.addCommands(turnLightGreen());
        auto.addCommands(new FollowPathCommand(follower, parkFromFarShoot()));
    }
        return auto;
    }
    private Command autoRoutineFar(){
        SequentialCommandGroup autoFar = new SequentialCommandGroup();

        //basic far auto commands
        autoFar.addCommands(new FollowPathCommand(follower, startToAprilTags()));
        autoFar.addCommands(new Delay(2.0));
        autoFar.addCommands(new FollowPathCommand(follower, aprilTagToShoot()));
        autoFar.addCommands(new FollowPathCommand(follower, shootToFlowerIntake()));
        autoFar.addCommands(new FollowPathCommand(follower, flowerToShoot()));
        autoFar.addCommands(new FollowPathCommand(follower, shootToPark()));

        return autoFar;
    }
    @Override
    public void init() {
        super.init();
        //Scheduler.reset();
        follower = Constants.create(hardwareMap);
        if (getStartZone() == StartZone.NEAR){
            follower.setPose(startNear);
        }
        else if(getStartZone() == StartZone.FAR){
            follower.setPose(startFar);
        }
    }
    @Override
    public void init_loop() {
        super.init_loop();
        readControls();

        //Stay on the near side during auto
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            noFarSide = true;
        } else if(operator.wasJustPressed(GamepadKeys.Button.B)){
            noFarSide = false;
        }// figure out how to make it toggle!
        //Bump the partner into the parking zone, then continue default path
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            pushToPark = true;
        } else if(operator.wasJustPressed(GamepadKeys.Button.A)){
            pushToPark = false;
        }

        telemetry.addData("Don't shoot on the other side (when near) \n Dpad up to turn on, b to turn off", noFarSide);
        telemetry.addData("Push partner. Dpad down to enable, A to Disable", pushToPark);

        telemetry.update();
    }
    @Override
    public void start() {
        if(getStartZone() == StartZone.NEAR){
            CommandScheduler.getInstance().schedule(autoRoutineNear());
        }
        else if(getStartZone() == StartZone.FAR){
            CommandScheduler.getInstance().schedule(autoRoutineFar());
        }
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
    }
}
