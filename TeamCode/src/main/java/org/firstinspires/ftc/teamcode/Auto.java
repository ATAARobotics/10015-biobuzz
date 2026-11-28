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
    //boolean pushToPark = false;
    boolean doubleCheck = true;
    //boolean basicNearAuto = false;
    //TODO:startposition
    private final Pose startUp = p.of(23.5 + xOffset, yOffset, 90);//Starting Pose for side where hive is tipped up p.s. sidelength of a tile on the edge is 23.5inch
    //private final Pose startMeet1 = p.of(19 - xOffset, 39 - yOffset, 0); //starting position up side
    private final Pose startDown = p.of(47 - xOffset, 141 - yOffset, 270); //starting position on down side
    //TODO:flower
  /*
  private final Pose controlPointSharedBetweenShootAndFlower = p.of(52, 85, 270); //control point between shoot position and flower intake
  private final Pose flowerIntakeFar = p.of(47.13, 126, 90); //flower intake down side (across from the hive)
  private final Pose farFlowerPickUp = p.of(46.4,122,90);//Pick up the flower at the far side
  private final Pose pointTowardsFlower = p.of(58,118.1,90); //rotate to point towards flower
  */
    //TODO:garden
    private final Pose gardenIntakeCorner = p.of(8.0,8,270); //getting into the corner for efficient garden intake
    private final Pose garden = p.of(10.0,9.14,260);//Pose to prepare to intake garden




    //TODO:shootposition
    private final Pose doubleCheckPartner = p.of(53+  xOffset, yOffset + 5.1, 270); //check if the hive has tipped or not
    private final Pose shootNear = p.of(23.5 + xOffset, yOffset+3, 90); //general shooting position up side
    //private final Pose shootFar = p.of(58,118,270);//Shoot at the far side
    //private final Pose shootOtherBasket = p.of(23.5+xOffset, 141.5 - yOffset - 6, 270); //shoot on the down side
    //private final Pose shootFarControlPoint = p.of(32.1,135.8,270); // control point when driving from garden to far shoot
    private final Pose checkAprilTag = p.of(72-xOffset, 127, 90); //waiting to see april tag (to know if hive has tipped)
    //private final Pose shootFar = p.of(60.96, 125, 270); old coordiantes for shootfar
    private final Pose shootFar = p.of(23.5+xOffset, 141.5 - yOffset - 6, 270);
    //TODO:park
    private final Pose park = p.of(13.0, 115.3, 270); //park down side TODO: parkMeet1 and parkFar can be the same?
    //private final Pose parkMeet1 = p.of(9.65+8, 117.57, 0); //parking position from down side //TODO: parkMeet1 and parkFar can be the same?
    //private final Pose parkMeet1Close = p.of(9.65+8, 105, 0); //parking position from up side
    //private final Pose endParkSolo = p.of(9.64,115,0); //Parking in zone at end alone
    //private final Pose controlParkFromGarden = p.of(41.09, 57.37, 90); //Control point for option where we don't go to other side
    //private final Pose parkOnEdge = p.of(8.25, 96.59,90); //Park closer to the edge to stay out of partner's way
    private final Pose prepareToBumpPartner = p.of(10.25,69.16,270); //So we don't destroy them
    //private final Pose controlPointToPrepareToBumpPartner = p.of(41.09,57.37,90); //Exactly what it sounds like
    private final Pose bumpPartner = p.of(10.25,96.33,270); //Push partner into park position


    //far auto poses (includes poses for meet 1 and general far auto poses)
    //meet 1 auto poses (includes doublechecking partner)


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
    class PushTimer extends CommandBase {
        double timeTarget;
        public PushTimer(double target) {
            this.timeTarget = target;
        }
        public boolean isFinished(){
            return timeTarget <= time;
        }
    }
    private Path goToNearShoot(){
        return line(startUp, shootNear).linear(startUp, shootNear);
    }
    private Path gardenPickUp() {
        Pose controlGarden = p.of(35.3, 36.9, 90); //Control point on way to pick up garden
        return curve(shootNear, controlGarden, garden).linear(shootNear, garden);
    }








    private Path pushPartner(){
        return line(prepareToBumpPartner, bumpPartner).linear(prepareToBumpPartner, bumpPartner);
    }




    //basic far auto paths
    private Path startToAprilTags(){
        return line(startDown, checkAprilTag).linear(startDown, checkAprilTag);
    }












    private Path gardenToDoubleCheck(){
        return line(gardenIntakeCorner, doubleCheckPartner).linear(gardenIntakeCorner, doubleCheckPartner);
    }




    private Path doubleCheckToShootNear(){
        return line(doubleCheckPartner, shootNear).linear(doubleCheckPartner, shootNear);
    }








    private Path doubleChecktoShootFar(){
        return line(doubleCheckPartner, shootFar).linear(doubleCheckPartner, shootFar);
    }




    private Path shootFarToPush(){
        Pose controlPointShootFarToPush = p.of(34.71, 55.86, 270);
        return curve(shootFar, controlPointShootFarToPush, prepareToBumpPartner).linear(shootFar, prepareToBumpPartner);
    }




    private Path shootNearToPark(){
        Pose controlPointLongPark = p.of(22.32, 23.61, 0);//control point for parking from up side
        return curve(doubleCheckPartner, controlPointLongPark, park).linear(doubleCheckPartner, park);
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




    private Path aprilTagToshootPreloads(){
        return line(checkAprilTag, shootFar).linear(checkAprilTag,shootFar);
    }
   /*private Path shootPreloadsToPark(){
       return line(shootFar, park).linear(shootFar,park);
   }
    */


    private Command autoRoutineUp() {
        SequentialCommandGroup auto = new SequentialCommandGroup();
        auto.addCommands(new InstantCommand(transfer::setCrashBarPollen));
        auto.addCommands(new FollowPathCommand(drive.follower, goToNearShoot()));
        auto.addCommands(new AutoShoot());
        auto.addCommands(new StartAutoIntake());
        auto.addCommands(new FollowPathCommand(drive.follower, gardenPickUp()));
        auto.addCommands(new Delay(0.5));
        auto.addCommands(new FollowPathCommand(drive.follower, gardenIntakePath()));
        auto.addCommands(new Delay(1.0));
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
                new FollowPathCommand(drive.follower, shootFarToPush()),
                new PushTimer(25),
                new FollowPathCommand(drive.follower, pushToPark())
                );
        SequentialCommandGroup notTipped = new SequentialCommandGroup();
        notTipped.addCommands(
                new FollowPathCommand(drive.follower, doubleCheckToShootNear()),
                new AutoShoot(),
                new Delay(0.5),
                new FollowPathCommand(drive.follower, shootNearToPreparePush()),
                new FollowPathCommand(drive.follower, pushPartner()),
                new PushTimer(25),
                new FollowPathCommand(drive.follower, pushToPark())
        );
        auto.addCommands(
                new ConditionalCommand(tipped, notTipped, () -> {return (tipDetector.tip() == TipDetector.TippedSide.UNKNOWN);})
        );
        return auto;
    }












    private Command autoRoutineDown(){




        //down auto for meet 1
        SequentialCommandGroup autoDown = new SequentialCommandGroup();
        autoDown.addCommands(new InstantCommand(transfer::setCrashBarPollen));




        autoDown.addCommands(new FollowPathCommand(drive.follower, startToAprilTags()));
        TipDetector.TippedSide target = TipDetector.TippedSide.SCORING;
        if(isRedAlliance()){
            target = TipDetector.TippedSide.SCORING;
        }
        else if(!isRedAlliance()){
            target = TipDetector.TippedSide.AUDIENCE;
        }
        autoDown.addCommands(new WaitForTip(target).withTimeout(20000));
        autoDown.addCommands(new FollowPathCommand(drive.follower, aprilTagToshootPreloads()));
        autoDown.addCommands(new AutoShoot());
        autoDown.addCommands(new PushTimer(25));
        autoDown.addCommands(new FollowPathCommand(drive.follower, shootFarToPush()));
        autoDown.addCommands(new FollowPathCommand(drive.follower, pushToPark()));






        // autoDown.addCommands(new FollowPathCommand(drive.follower, shootPreloadsToPark()));




        return autoDown;




      /*
      //TODO:Desired outcome for far auto, uses flowers as well
      autoDown.addCommands(new FollowPathCommand(drive.follower, startToAprilTags()));
      autoDown.addCommands(new WaitForTip(TipDetector.TippedSide.SCORING));
      autoDown.addCommands(new FollowPathCommand(drive.follower, aprilTagToShoot()));
      autoDown.addCommands(new FollowPathCommand(drive.follower, shootToFlowerIntake()));
      autoDown.addCommands(new FollowPathCommand(drive.follower, flowerToShoot()));
      autoDown.addCommands(new FollowPathCommand(drive.follower, shootToPark()));




      return autoDown;




       */
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
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            doubleCheck = true;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.Y)){
            doubleCheck = false;
        }




        //  telemetry.addData("Don't shoot on the other side (when upside) \n Dpad up to turn on, b to turn off", noFarSide);
        // telemetry.addData("Push partner. DPAD DOWN to enable, A to disable", pushToPark);
        //   telemetry.addData("Meet 1 with partner. DPAD LEFT to enable, Y to disable.", doubleCheck);




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



