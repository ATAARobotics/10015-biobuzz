package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
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
    
    private final PoseFactory p = PoseFactory.degrees();
    double xOffset = 8.124;
    double yOffset = 8.649;
    boolean noFarSide = false;
    boolean pushToPark = false;
    boolean doubleCheck = true;
    boolean basicNearAuto = false;
    boolean pushIntoZone = false;
    private final Pose startUp = p.of(47 + xOffset, yOffset, 90);//Starting Pose for side where hive is tipped up
    private final Pose garden = p.of(10.0,9.14,260);//Pose to prepare to intake garden
    private final Pose shootNear = p.of(47 + xOffset, yOffset+3, 90);
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
    private final Pose startDown = p.of(72 - xOffset, 141 - yOffset, 270);
    private final Pose checkAprilTag = p.of(72-xOffset, (141 - yOffset) + 0.2, 270);
    private final Pose generalShootingPositionFar = p.of(60.96, 132.4, 270);
    private final Pose controlPointSharedBetweenShootAndFlower = p.of(52, 85, 270);
    private final Pose flowerIntakeFar = p.of(47.13, 126, 90);
    private final Pose parkFar = p.of(13.0, 115.3, 270);
    private final Pose controlPointParkFar = p.of(63.5, 99.35, 270);

    //meet 1, doublecheck auto poses
    private final Pose startNoGarden = p.of(19 - xOffset, 39 - yOffset, 0);
    private final Pose doubleCheckPartner = p.of(58 + xOffset, yOffset + 5.1, 270);
    private final Pose shootInCase = p.of(58.37, 17, 90);
    private final Pose shootOtherBasket = p.of(59.71, 115, 270);
    private final Pose parkMeet1 = p.of(9.65+8, 117.57, 0);
    private final Pose controlPointParkMeet1 = p.of(40.49, 94.98, 0);
    private final Pose controlPointLongPark = p.of(22.32, 23.61, 0);
    private final Pose gardenIntakeCorner = p.of(8.0,8,270);
    private final Pose parkMeet1Close = p.of(9.65+8, 105, 0);

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
    class WaitForTip extends CommandBase {
        TipDetector.TippedSide target;
        public WaitForTip(TipDetector.TippedSide target) {
            this.target = target;
        }
        public boolean isFinished(){
            return tipDetector.tip() == target;
        }
    }
    private Path goToNearShoot(){
        return line(startUp, shootNear).linear(startUp, shootNear);
    }
    private Path gardenPickUp() {
        return curve(shootNear, controlGarden, garden).linear(shootNear, garden);
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
        return line(startDown, checkAprilTag).linear(startDown, checkAprilTag);
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

    private Path gardenToDoubleCheck(){
        return line(gardenIntakeCorner, doubleCheckPartner).linear(gardenIntakeCorner, doubleCheckPartner);
    }
    private Path startToDoubleCheck(){
        return line(startNoGarden, doubleCheckPartner).linear(startNoGarden, doubleCheckPartner);
    }
    private Path doubleCheckToShootNear(){
        return line(doubleCheckPartner, shootNear).linear(doubleCheckPartner, shootNear);
    }
    private Path shootNearToDoubleCheck(){
        return line(shootNear, doubleCheckPartner).linear(shootNear, doubleCheckPartner);
    }

    private Path doubleChecktoShootFar(){
        return line(doubleCheckPartner, shootOtherBasket).linear(doubleCheckPartner, shootOtherBasket);
    }

    private Path shootFarToPark(){
        return curve(shootOtherBasket, controlPointParkMeet1, parkMeet1).linear(shootOtherBasket, parkMeet1);
    }

    private Path shootNearToPark(){
        return curve(doubleCheckPartner, controlPointLongPark, parkMeet1Close).linear(doubleCheckPartner, parkMeet1Close);
    }

    private Path gardenIntakePath(){
        return line(garden,gardenIntakeCorner).linear(garden,gardenIntakeCorner);
    }
    private Path shootNearToPreparePush(){
        return line(shootNear,prepareToBumpPartner).linear(shootNear,prepareToBumpPartner);
    }
    private Path pushToPark(){
        return line(prepareToBumpPartner, bumpPartner).linear(prepareToBumpPartner,bumpPartner);
    }
    private Command autoRoutineUp() {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        auto.addCommands(new InstantCommand(transfer::setCrashBarPollen));
        auto.addCommands(new FollowPathCommand(drive.follower, goToNearShoot()));
	auto.addCommands(new AutoShoot());
	auto.addCommands(new StartAutoIntake());
	auto.addCommands(new FollowPathCommand(drive.follower, gardenPickUp()));
    auto.addCommands(new Delay(1.0));
        auto.addCommands(new FollowPathCommand(drive.follower, gardenIntakePath()));
	auto.addCommands(new Delay(2.0));
	auto.addCommands(new StopAutoIntake());
    auto.addCommands(new InstantCommand(transfer::setCrashBarNothing));
	auto.addCommands(new FollowPathCommand(drive.follower, gardenToDoubleCheck()));
    auto.addCommands(new InstantCommand(transfer::setCrashBarPollen));

	// once at double-check location we decide whether to shoot
	// from here, or drive to other side
	SequentialCommandGroup tipped = new SequentialCommandGroup();
	tipped.addCommands(
		new FollowPathCommand(drive.follower, doubleChecktoShootFar()),
		// should we also wait here??
		//new WaitForTip(TipDetector.TippedSide.SCORING),
		new AutoShoot(),
		new Delay(0.5),
		new FollowPathCommand(drive.follower, shootFarToPark())
	);
	SequentialCommandGroup notTipped = new SequentialCommandGroup();
	notTipped.addCommands(
		new FollowPathCommand(drive.follower, doubleCheckToShootNear()),
		new AutoShoot(),
		new Delay(0.5)
    );

    if(pushIntoZone){
        notTipped.addCommands(
		new FollowPathCommand(drive.follower, shootNearToPreparePush()),
            new FollowPathCommand(drive.follower, pushPartner()),
        new FollowPathCommand(drive.follower, pushToPark())
            );
        }
    else{
        notTipped.addCommands(
            new FollowPathCommand(drive.follower, shootNearToPark())
            );
        }
	auto.addCommands(
		new ConditionalCommand(tipped, notTipped, () -> {return (tipDetector.tip() == TipDetector.TippedSide.UNKNOWN);})
	);
	return auto;
    }

    public Command oldUpAuto() {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        if (false) {///basicUpAuto) {
            //Shoot preloads first from starting position
            // auto.addCommands(turnLightGreen());
            auto.addCommands(new FollowPathCommand(drive.follower, gardenPickUp()));
            //run intake
            //   auto.addCommands(turnLightRed());
            if (noFarSide) {
                auto.addCommands(new FollowPathCommand(drive.follower, parkFromGarden()));
            } else {
                if (pushToPark) {
                    auto.addCommands(new FollowPathCommand(drive.follower, prepareToPush()));
                    //add delay here?
                    auto.addCommands(new FollowPathCommand(drive.follower, pushPartner()));
                    auto.addCommands(new FollowPathCommand(drive.follower, bumpToShoot())); //Going from parking to far shoot
                } else {
                    auto.addCommands(new FollowPathCommand(drive.follower, goToFarShoot())); //Going from garden to far shoot
                    auto.addCommands(new Delay(1.0));
                }
                //Shoot
                //    auto.addCommands(turnLightGreen());
                auto.addCommands(new FollowPathCommand(drive.follower, turnToFlower()));
                auto.addCommands(new FollowPathCommand(drive.follower, pickUpFarFlower()));
                auto.addCommands(new Delay(1));
                //intake flower
                //  auto.addCommands(turnLightRed());
                auto.addCommands(new FollowPathCommand(drive.follower, shootFromFarFlower()));
                //shoot
                //   auto.addCommands(turnLightGreen());
                auto.addCommands(new FollowPathCommand(drive.follower, parkFromFarShoot()));
            }
        }
        return auto;
    }

    private Command autoRoutineDown(){
        SequentialCommandGroup autoDown = new SequentialCommandGroup();
        autoDown.addCommands(new InstantCommand(transfer::setCrashBarPollen));

        //basic far auto commands
        autoDown.addCommands(new FollowPathCommand(drive.follower, startToAprilTags()));
        autoDown.addCommands(new WaitForTip(TipDetector.TippedSide.SCORING));
        autoDown.addCommands(new FollowPathCommand(drive.follower, aprilTagToShoot()));
        autoDown.addCommands(new FollowPathCommand(drive.follower, shootToFlowerIntake()));
        autoDown.addCommands(new FollowPathCommand(drive.follower, flowerToShoot()));
        autoDown.addCommands(new FollowPathCommand(drive.follower, shootToPark()));

        return autoDown;
    }

    private final Pose testStart = p.of(73-xOffset, yOffset, 90);
    private final Pose testEnd = p.of(73-xOffset, 120, 90);
    private Path testPath(){
        return line(testStart, testEnd).linear(testStart, testEnd);
    }
    private Command test(){
        SequentialCommandGroup testAuto = new SequentialCommandGroup();
        testAuto.addCommands(new FollowPathCommand(drive.follower, testPath()));
        return testAuto;

    }
    @Override
    public void init() {
        super.init();
        //Scheduler.reset();
        drive.follower = Constants.create(hardwareMap);
        if (getStartZone() == StartZone.UP){
            drive.follower.setPose(startUp);
        }
        else if(getStartZone() == StartZone.DOWN){
            drive.follower.setPose(startDown);
        }
        //drive.follower.setPose(testStart);
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
   /*     if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            pushToPark = true;
        } else if(operator.wasJustPressed(GamepadKeys.Button.A)){
            pushToPark = false;
        }

    */
        if(operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            pushIntoZone = true;
        }
        if(operator.wasJustPressed(GamepadKeys.Button.A)){
            pushIntoZone = false;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            doubleCheck = true;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.Y)){
            doubleCheck = false;
        }

        telemetry.addData("Don't shoot on the other side (when upside) \n Dpad up to turn on, b to turn off", noFarSide);
       // telemetry.addData("Push partner. DPAD DOWN to enable, A to disable", pushToPark);
        telemetry.addData("Meet 1 with partner. DPAD LEFT to enable, Y to disable.", doubleCheck);
        telemetry.addData("Push partner. DPAD DOWN to enable, A to disable", pushIntoZone);

        telemetry.update();
    }
    @Override
    public void start() {
        super.start();
        OpmodeStorage.autoRan=true;
        if(getStartZone() == StartZone.UP){
            CommandScheduler.getInstance().schedule(autoRoutineUp());
        }
        else if(getStartZone() == StartZone.DOWN){
            CommandScheduler.getInstance().schedule(autoRoutineDown());
        }
      //  CommandScheduler.getInstance().schedule(test());
    }

    @Override
    public void loop() {
        super.loop();
        drive.follower.update();
    }

    @Override
    public void stop(){
        OpmodeStorage.autonomousEndPose = drive.follower.pose(); //saves your position in that file
    }
}
