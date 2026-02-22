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
    private boolean gatePickUp = false:

    private final Pose blueFarStart = new Pose(47.25 + xOffset, yOffset, Math.toRadians(90));
    private final Pose blueNearStart = new Pose(19.5 + xOffset,120.5 + yOffset, Math.toRadians(90));
    private final Pose blueFarShoot = new Pose(
            47.5 + 8.124 + 2.0, // 2 inches further towards Red from start
            yOffset + 10.0,// 10 inches in front of start position
            Math.toRadians(110)
        );
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
    private final Pose thirdBlueNearShoot = new Pose (55, 73, Math.toRadians(180));

    private final Pose spikeStart1 = new Pose (50, 35.0, Math.toRadians(180));
    private final Pose spikeEnd1 = new Pose (10.4, 35.0, Math.toRadians(180));

    // next set of spikes is one tile away
    private final Pose spikeStart2 = new Pose (50, 35.0 + 23.5, Math.toRadians(180));
    private final Pose spikeEnd2 = new Pose (10.4, 35.0 + 23.5, Math.toRadians(180));

    //curve between spikeEnd2 and thirdBlueNearShoot
    private final Pose controlPoint1 = new Pose (56.1, 48.4);

    // closest set of spikes has the ramp in the way so we can't drive as far forward
    private final Pose spikeStart3 = new Pose (50, 35.0 + (2 * 23.5), Math.toRadians(180));
    private final Pose spikeEnd3 = new Pose (10.4 + 6.0, 35.0 + (2 * 23.5), Math.toRadians(180));
    private final Pose gateOpen0= new Pose (25, 35.0 + (2 * 23.5) - 4, Math.toRadians(180));
    private final Pose gateOpen1 = new Pose (10.4 +6 , 35.0 + (2 * 23.5) - 4, Math.toRadians(180));
    private final Pose gatePickUpStart = new Pose(11.42, 23, Math.toRadians(130));
    private final Pose gatePickUpEnd = new Pose(11.42, 55, Math.toRadians(130));
    private final Pose nearParkGate = new Pose (36.0, 72, Math.toRadians(180));

    // human-player preloads
    private final Pose wallFar = new Pose(xOffset + 4, 23.6 + yOffset, Math.toRadians(235));
    private final Pose wallClose = new Pose(xOffset + 4, yOffset + 3, Math.toRadians(255));
    private final Pose wallClosish = new Pose(xOffset + 4, yOffset + 5, Math.toRadians(270));

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
    public Command pathBetween(Pose b, Pose e, double speed) {
        Pose begin = convert(b);
        Pose end = convert(e);
        PathChain p = follower.pathBuilder()
            .setGlobalDeceleration()
            .addPath(new BezierLine(begin, end))
            .setLinearHeadingInterpolation(begin.getHeading(), end.getHeading())
            .build();
        return new FollowPathCommand(p, speed);
    }

    public Command curveBetween(Pose b, Pose control, Pose e, double speed) {
        Pose begin = convert(b);
        Pose end = convert(e);
        PathChain p = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierCurve(begin, control, end))
                .setLinearHeadingInterpolation(begin.getHeading(), end.getHeading())
                .build();
        return new FollowPathCommand(p, speed);
    }

    class FollowPathCommand extends CommandBase {
        PathChain path;
        double speed;

        public FollowPathCommand(PathChain p, double s) {
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

        auto.addCommands(
            new ParallelCommandGroup(
                new SortSpindex(),
                pathBetween(blueFarStart, blueFarShoot, 1.0)
            )
        );
        if (usePreloads) {
            auto.addCommands(new AutoOuttake(),
            new Delay(0.5));

        }

        // collect and shoot audience spike mark
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
                pathBetween(spikeEnd1, blueFarShoot, 1.0)
            ),
            new AutoOuttake()
        );

        // collect and shoot human-player preloads
        auto.addCommands(
            pathBetween(blueFarShoot, wallFar, 1.0),
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
                new SortSpindex(),
                pathBetween(wallClose, blueFarShoot, 1.0)
            ),
            new AutoOuttake(),
            new Delay(0.5)
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
        Pose lastGate = blueFarShoot;
        if (gatePickUp) {
            auto.addCommands(
                    pathBetween(blueFarStart, gatePickUpStart, 1.0),
                    new ParallelRaceGroup(
                            new AutoIntake(),
                            pathBetween(gatePickUpStart, gatePickUpEnd, 0.4)
                    )
            );
            lastGate = gatePickUpEnd;
        }
        // park off the start lines
        auto.addCommands(
            pathBetween(lastGate, blueFarPark, 1.0)
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
        Pose spikeStart = blueNearShoot;
        if (usePreloads) {
            auto.addCommands(
                new LookAtObelisk(),
                new PrepareToShoot(),
                new ParallelCommandGroup(
                    new SortSpindex(),
                    pathBetween(blueNearStart, blueNearShoot, 1.0)
                ),
                new AutoOuttake(),
                    new Delay(0.5)
                );
        } else {
            spikeStart = blueNearStart;
        }

        // pick up and shoot far spike mark (note our start position
        // depends on whether usePreloads was active or not)
        auto.addCommands(
                pathBetween(spikeStart, spikeStart3, 1.0),
                new ParallelRaceGroup(
                    new AutoIntake(),
                    pathBetween(spikeStart3, spikeEnd3, 0.35)
                ),
                new SoftIntake(),
                new Delay(1)
        );
        Pose beforeGate = spikeEnd3;
        if (closeGateOpen) {
            auto.addCommands(
                    pathBetween(spikeEnd3, gateOpen0, 1.0),
                    pathBetween(gateOpen0, gateOpen1, 0.8),
                    new Delay(0.5)
            );
            beforeGate = gateOpen1;
        }

        // shooting spike three after opening gate
        auto.addCommands(
                new PrepareToShoot(),
                new ParallelCommandGroup(
                    new SortSpindex(),
                    pathBetween(beforeGate, secondBlueNearShoot, 1.0)
                ),
                new AutoOuttake(),
                new Delay(0.5)
        );
        // pick up and shoot middle spike mark
        auto.addCommands(
                pathBetween(secondBlueNearShoot, spikeStart2, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart2, spikeEnd2, 0.35)
                ),
                new SoftIntake(),
                new Delay(1)
                );

        Pose lastSpike = spikeEnd2;
        if (whenOpenGate == 1) {
            // open the gate after picking up spike 3
            auto.addCommands(
                    pathBetween(spikeEnd2, openGate, 1.0)
            );
            lastSpike = openGate;
        }

        auto.addCommands(
                new PrepareToShoot(),
                new ParallelCommandGroup(
                    new SortSpindex(),
                    curveBetween(lastSpike, controlPoint1, thirdBlueNearShoot, 1.0)
                ),
                new AutoOuttake(),
                new Delay(0.5)
        );

        if (audienceSpike){
            auto.addCommands(
                    pathBetween(thirdBlueNearShoot, spikeStart1, 1.0),
                    new ParallelRaceGroup(
                            new AutoIntake(),
                            pathBetween(spikeStart1, spikeEnd1, 0.35)
                    ),
                    new SoftIntake(),
                    new Delay(1),
                    new PrepareToShoot(),
                    new ParallelCommandGroup(
                            new SortSpindex(),
                            pathBetween(spikeEnd1, thirdBlueNearShoot, 1.0)
                    ),
                    new AutoOuttake(),
                    new Delay(0.5),
                    pathBetween(thirdBlueNearShoot, nearParkGate, 1.0)
            );
        }
        else {
            auto.addCommands(
                    pathBetween(thirdBlueNearShoot, nearParkGate, 1.0));
        }

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
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            audienceSpike = !audienceSpike;
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            closeGateOpen = !closeGateOpen;
        }

        String openDescription = "unknown";
        if (whenOpenGate == 0) openDescription = "Never";
        if (whenOpenGate == 1) openDescription = "After Spike3 pickup";


        telemetry.addData("Preloads (Dpad Up to toggle)", usePreloads);
      //  telemetry.addData("Open Gate (Dpad Down to toggle)", openDescription);
        // telemetry.addData("Pick up third spike (Dpad Left to toggle", audienceSpike);
        telemetry.addData("Intake from gate far zone (Dpad Left to toggle)", gatePickUp);
        telemetry.addData("Near zone open gate (Dpad Right to toggle)", closeGateOpen);
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
        editor.putFloat("turret", (float)turret.getOriginalResetAngle());
        editor.putInt("spindex-target", spindexer.targetAngle);
        editor.apply();

        super.stop();
    }
}
