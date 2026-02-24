package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierPoint;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Consumer;
import java.util.function.Function;

import kotlinx.coroutines.Delay;


public abstract class Auto extends RobotBaseOp {
    public Follower follower;
    double xOffset = 8.124;
    double yOffset = 8.0984;
    private boolean usePreloads = true;
    private int whenOpenGate = 0;
    private boolean audienceSpike = false;
    private boolean closeGateOpen = false;
    private boolean gatePickUp = true;
    private boolean gateIntake2 = true;

    private final Pose blueFarStart = new Pose(47.25 + xOffset, yOffset, Math.toRadians(90));
    //private final Pose blueNearStart = new Pose(19.5 + xOffset,120.5 + yOffset, Math.toRadians(270));
    private final Pose blueNearStart = new Pose(23.25 + xOffset, 140 - yOffset, Math.toRadians(270));
    private final Pose blueFarShoot = new Pose(
            47.5 + 8.124,
            yOffset + 10.0,// 10 inches in front of start position
            Math.toRadians(180)
        );
    // just inside box, slightly closer to spike1
    private final Pose blueFarShoot2 = new Pose(60, 24.36, Math.toRadians(180));
    private final Pose blueFarPark = new Pose(
            47.5 + xOffset, // same as start offset
            yOffset + 25.0,// 25 inches in front of start position
            Math.toRadians(90)
        );
    private final Pose blueNearPark = new Pose(23.5 + xOffset, 23.5*3, Math.toRadians(270));
    private final Pose blueNearShoot = new Pose(
            45, // stay inside our side of the field
            105.0, // right at top of the cone
            Math.toRadians(180)
        );
    private final Pose secondBlueNearShoot = new Pose (50, 102, Math.toRadians(180));
    private final Pose thirdBlueNearShoot = new Pose (55, 83, Math.toRadians(180));
    private final Pose finalShootAndPark = new Pose (56, 111, Math.toRadians(180));

    // these are red coords
    //private final Pose gateIntake = new Pose(129.14, 58.7, Math.toRadians(30));
    private final Pose gatePreIntake = new Pose(13.0, 58.7, Math.toRadians(150));
    private final Pose gateIntake = new Pose(11.75, 58.7, Math.toRadians(150));
    private final Pose gateIntakeBack = new Pose(11.75, 53.0, Math.toRadians(150));
    private final Pose gateControl = new Pose(30.0, 58.7, Math.toRadians(180));

    private final Pose spikeStart1 = new Pose (42, 35.0, Math.toRadians(180));
    private final Pose spikeEnd1 = new Pose (10.4, 35.0, Math.toRadians(180));
    private final Pose spikeStart1Control = new Pose(57.4, 37.27, Math.toRadians(180));

    // next set of spikes is one tile away
    private final Pose spikeStart2 = new Pose (43, 35.0 + 23.5, Math.toRadians(180));
    private final Pose spikeEnd2 = new Pose (10.4, 35.0 + 23.5, Math.toRadians(180));

    //curve between spikeEnd2 and thirdBlueNearShoot
    private final Pose controlPoint1 = new Pose (56.1, 48.4);

    // closest set of spikes has the ramp in the way so we can't drive as far forward
    private final Pose spikeStart3 = new Pose (50, 35.0 + (2 * 23.5), Math.toRadians(180));
    private final Pose spikeEnd3 = new Pose (10.4 + 6.0, 35.0 + (2 * 23.5), Math.toRadians(180));
    private final Pose gateOpen0= new Pose (25, 35.0 + (2 * 23.5) - 4, Math.toRadians(180));
    private final Pose gateOpen1 = new Pose (10.4 +6 , 35.0 + (2 * 23.5) - 4, Math.toRadians(180));
    private final Pose gatePickUpStart = new Pose(11.42, 23, Math.toRadians(125));
    private final Pose gatePickUpEnd = new Pose(11.42, 50, Math.toRadians(125));
    private final Pose nearParkGate = new Pose (36.0, 72, Math.toRadians(180));

    // human-player preloads
    private final Pose wallFar = new Pose(xOffset + 5, 23.6 + yOffset, Math.toRadians(210));
    private final Pose wallControl = new Pose(57.624, 23.6 + yOffset, Math.toRadians(270));
    private final Pose wallClose = new Pose(xOffset + 4, yOffset + 3, Math.toRadians(255));
    private final Pose wallClosish = new Pose(xOffset + 4, yOffset + 5, Math.toRadians(270));

    private final Pose humanPlayerIntake = new Pose(9.8, 8.6,Math.toRadians(180));
    private final Pose humanPlayerIntakeTwo = new Pose(11.42,23,Math.toRadians(180));
    private final Pose secretTunnelIntake = new Pose(11.42,43,130);
    private final Pose theCorner = new Pose(yOffset, xOffset, Math.toRadians(180));
    // trying a different human-player routing
    private final Pose wallDirectStart = new Pose(xOffset + 1, yOffset + 15, Math.toRadians(240));
    private final Pose wallDirectEnd = new Pose(xOffset + 1, yOffset, Math.toRadians(240));

    private final Pose openGate = new Pose(10.4 + 7.0, 70, Math.toRadians(90));


    protected void bindDriverControls() {}
    protected void bindOperatorControls() {}
    public boolean isAuto() { return true; }

    private Command _lastCommandRun = null;

    private Pose convert(Pose blue){
        if (getAlliance() == Alliance.BLUE){
            return blue;
        }
        else {
            return blueToRed(blue);
        }
    }
    public FollowPathCommand pathBetween(Pose b, Pose e, double speed) {
        Pose begin = convert(b);
        Pose end = convert(e);
        Path p = new Path(new BezierLine(begin, end));
        if (begin.getHeading() != end.getHeading()) {
            p.setLinearHeadingInterpolation(begin.getHeading(), end.getHeading());
        } else {
            p.setConstantHeadingInterpolation(end.getHeading());
        }
        // via Brogan M Pratt, default is 1.0 .. lower numbers stop SOONER.
        // brakingStart default is 1
        ///p.setBrakingStrength(1.2);
        return new FollowPathCommand(p, speed);
    }
    public FollowPathCommand curveBetween(Pose b, Pose c,Pose e,  double speed) {
        Pose begin = convert(b);
        Pose end = convert(e);
        Pose control = convert(c);
        Path p = new Path(new BezierCurve(begin, control, end));

        if (begin.getHeading() != end.getHeading()) {
            p.setLinearHeadingInterpolation(begin.getHeading(), end.getHeading());
        } else {
            p.setConstantHeadingInterpolation(end.getHeading());
        }
        //p.setBrakingStrength(1.2);  // same as passing in setGlobalDeceleration()
        return new FollowPathCommand(p, speed);
    }

    class FollowPathCommand extends CommandBase {
        Path path;
        double speed;

        public FollowPathCommand(Path p, double s) {
            path = p;
            speed = s;
            addRequirements(drive);
        }
        public void initialize() {
            follower.followPath(path);
            follower.setMaxPower(speed);
        }
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

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

    private Pose blueToRed(Pose blue){
        double h = 180 - Math.toDegrees(blue.getHeading());
        if (h < 0) {
            h = h + 360;
        }
        Pose red = new Pose(141 - blue.getX(), blue.getY(), Math.toRadians(h));
        return red;
    }

    //
    // far-zone auto pathing
    //

    private Command farPathing() {
        Pose start = convert(blueFarStart);
        drive.setPosition(
            new Pose2D(
                DistanceUnit.INCH,
                start.getX(),
                start.getY(),
                AngleUnit.RADIANS,
                start.getHeading()
            )
        );
        follower.setStartingPose(start);

        SequentialCommandGroup auto = new SequentialCommandGroup();

        if (usePreloads) {
            auto.addCommands(new PrepareToShoot());
        }

        FollowPathCommand fp = pathBetween(blueFarStart, blueFarShoot, 1.0);
        ///if (!usePreloads) fp.path.setNoDeceleration();
        auto.addCommands(
            new ParallelCommandGroup(
                new SortSpindex(),
                fp
            )
        );
        if (usePreloads) {
            auto.addCommands(new AutoOuttake(),
            new Delay(0.250));

        }

        // collect and shoot audience spike mark
        fp = pathBetween(spikeEnd1, blueFarShoot2, 1.0);
        //fp.path.
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart1, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                pathBetween(spikeStart1, spikeEnd1, 0.45)
            ),
            new SoftIntake(),
            new Delay(1.0),
            new PrepareToShoot(),
            new ParallelCommandGroup(
                new SortSpindex(),
                new PrepareToShoot(),
                fp
            ),
            new AutoOuttake(),
            new Delay(0.250)
        );

        // collect and shoot human-player preloads
        auto.addCommands(
            curveBetween(blueFarShoot, wallControl, wallFar,  1.0), //To do: add in bezier curve
            new ParallelRaceGroup(
                new AutoIntake(),
                new SequentialCommandGroup(
                    pathBetween(wallFar, wallClose, 0.55),
                    new Delay(1.0),
                        pathBetween(wallClose, wallClosish, 0.55),
                        new Delay(1.0)
                )
            ),
            new SoftIntake(),
            new PrepareToShoot(),
            new ParallelCommandGroup(
                new PrepareToShoot(),
                new SortSpindex(),
                pathBetween(wallClose, blueFarShoot, 1.0)
            ),
            new AutoOuttake(),
            new Delay(0.250)
        );

/*
        // collect and shoot middle spike mark
        auto.addCommands(
            pathBetween(blueFarShoot, spikeStart2, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                pathBetween(spikeStart2, spikeEnd2, 0.45)
            ),
            pathBetween(spikeEnd2, blueFarShoot, 1.0),
            new AutoOuttake()
        );
*/

        if (gatePickUp) {
            auto.addCommands(
                new ParallelRaceGroup(
                    new ParallelCommandGroup(
                        pathBetween(blueFarShoot, theCorner, 0.55),
                        new Delay(0.500)
                    ),
                    new AutoIntake()
                ),
                new ParallelCommandGroup(
                    new PrepareToShoot(),
                    new SortSpindex(),
                    pathBetween(theCorner, blueFarShoot, 1.0)
                ),
                new AutoOuttake(),
                new Delay(0.1)
            );
        }

        // park off the start lines
        auto.addCommands(
            pathBetween(blueFarShoot, blueFarPark, 1.0)
        );

        auto.addCommands(
            new SpindexMode()
        );

        return auto;
    }


    //
    // near-zone auto pathing
    //

    private Command nearPathing() {
        Pose start = convert(blueNearStart);
        drive.setPosition(
            new Pose2D(
                DistanceUnit.INCH,
                start.getX(),
                start.getY(),
                AngleUnit.RADIANS,
                start.getHeading()
            )
        );
        follower.setStartingPose(start);

        SequentialCommandGroup auto = new SequentialCommandGroup();

        // shoot preloads
        Pose spikeStart = secondBlueNearShoot;
        if (usePreloads) {
            auto.addCommands(
                new PrepareToShoot(),
                pathBetween(blueNearStart, secondBlueNearShoot, 1.0),
                new AutoOuttake(),
                new Delay(0.100)
           );
        } else {
            spikeStart = blueNearStart;
        }

        // pick up and shoot middle spike mark
        auto.addCommands(
            pathBetween(secondBlueNearShoot, spikeStart2, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        pathBetween(spikeStart2, spikeEnd2, 0.35),
                        new LookAtObelisk()
                    ),
                    new Delay(0.2)
                )
            ),
            new SoftIntake()
        );

        auto.addCommands(
            new PrepareToShoot(),
            new ParallelCommandGroup(
                new SortSpindex(),
                curveBetween(spikeEnd2, spikeStart2, thirdBlueNearShoot, 1.0)
            ),
            new Delay(0.100),
            new AutoOuttake(),
            new Delay(0.1)
        );

        // open gate plus intake from ramp
        auto.addCommands(
            curveBetween(thirdBlueNearShoot, gateControl, gatePreIntake, 0.9),
            new ParallelRaceGroup(
                new AutoIntake(),
                new SequentialCommandGroup(
                    pathBetween(gatePreIntake, gateIntake, 0.7),
                    new Delay(0.650),
                    pathBetween(gateIntake, gateIntakeBack, 0.35)
                )
            ),
            new SoftIntake(),
            new ParallelCommandGroup(
                new PrepareToShoot(),
                new ParallelCommandGroup(
                    new SortSpindex(),
                    pathBetween(gateIntakeBack, thirdBlueNearShoot, 1.0)
                )
            ),
            new Delay(0.100),
            new AutoOuttake()
        );
        //gate intake 2
        if (gateIntake2) {
            auto.addCommands(
                curveBetween(thirdBlueNearShoot, gateControl, gatePreIntake, 0.9),
                new ParallelRaceGroup(
                    new AutoIntake(),
                    new SequentialCommandGroup(
                        pathBetween(gatePreIntake, gateIntake, 0.7),
                        new Delay(0.650),
                        pathBetween(gateIntake, gateIntakeBack, 0.35)
                    )
                ),
                new SoftIntake(),
                new ParallelCommandGroup(
                    new PrepareToShoot(),
                    new ParallelCommandGroup(
                        new SortSpindex(),
                        pathBetween(gateIntakeBack, thirdBlueNearShoot, 1.0)
                    )
                ),
                new Delay(0.1),
                new AutoOuttake()
            );
        }

        // intake spike 3 (further from audience)
        auto.addCommands(
            pathBetween(thirdBlueNearShoot, spikeStart3, 1.0),
            new ParallelRaceGroup(
                new AutoIntake(),
                pathBetween(spikeStart3, spikeEnd3, 0.35)
                ),
            new SoftIntake(),
            new PrepareToShoot(),
            new ParallelCommandGroup(
                new SortSpindex(),
                pathBetween(spikeEnd3, finalShootAndPark, 1.0)
            ),
            new Delay(0.100),
            new AutoOuttake()
        );


        auto.addCommands(
            new SpindexMode()
        );

        return auto;
    }

    @Override
    public void init(){
        super.init();
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        readControls();

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            usePreloads = !usePreloads;
        }
      /*  if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            whenOpenGate += 1;
            if (whenOpenGate > 1) {
                whenOpenGate = 0;
            }
        }*/
     /*   if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            audienceSpike = !audienceSpike;
        }*/
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            closeGateOpen = !closeGateOpen;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            gateIntake2 = true;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            gatePickUp = true;
        }

       // if (whenOpenGate == 0) openDescription = "Never";
       // if (whenOpenGate == 1) openDescription = "After Spike3 pickup";


        telemetry.addData("Preloads (Dpad Up to toggle)", usePreloads);
      //  telemetry.addData("Open Gate (Dpad Down to toggle)", openDescription);
        // telemetry.addData("Pick up third spike (Dpad Left to toggle", audienceSpike);
        telemetry.addData("Intake from gate far zone (Dpad Left to toggle)", gatePickUp);
        telemetry.addData("Near zone open gate (Dpad Right to toggle)", closeGateOpen);
        telemetry.addData("Near zone 2nd gate intake (Dpad Down to toggle)", gateIntake2);
        telemetry.update();
    }

    @Override
    public void start() {
        // we must run this _before_ the "pathing" options because
        // those set the start-position of the robot
        super.start();

        // "Tuning.Line" tuner does this .. really needed?
        follower.activateAllPIDFs();

        Command cmds = null;
        switch (getStartZone()) {
            case NEAR:
                cmds = nearPathing();
                break;
            case FAR:
                cmds = farPathing();
                break;
        }

        CommandScheduler.getInstance().schedule(cmds);

        // tell the Spindexer about its preloads
        if (usePreloads) {
            spindexer.slots[0] = Spindexer.SlotContent.Green;
            spindexer.slots[1] = Spindexer.SlotContent.Purple;
            spindexer.slots[2] = Spindexer.SlotContent.Purple;
        }
    }

    @Override
    protected void addTelemetry(HyperTelemetry telem) {

        super.addTelemetry(telem);
        telem.logBoth("Pinpoint-status", drive.pinpoint.getDeviceStatus());
    }

    @Override
    public void loop(){
        follower.update();
        // the command-scheduler is run in our super-class
        super.loop();
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.INCH));
        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.INCH));
        editor.putFloat("turret", (float)turret.ticks);
        editor.putInt("spindex-target", spindexer.targetAngle);
        editor.putInt("obelisk", turret.pattern);
        editor.apply();

        super.stop();
    }
}
