package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public abstract class Auto extends RobotBaseOp {
    public Follower follower;
    double xOffset = 7.179;
    double yOffset = 7.19;
    private boolean usePreloads = true;
    private boolean soloNear = false;
    private int whenOpenGate = 0;
    private boolean audienceSpike = false;
    private boolean closeGateOpen = false;
    private boolean gatePickUp = true;
    private boolean gateIntake2 = true;

    public static double GATE_X = 11.67;
    public static double GATE_Y = 60.0;
    public static double GATE_HEADING = 150.0;

    // feb 27 changed start to be 180 degrees and sideways so new
    private final Pose blueFarStart = new Pose(70 - yOffset, xOffset, Math.toRadians(180));
    //private final Pose blueFarStart = new Pose(47.25 + xOffset, yOffset, Math.toRadians(90));
    //private final Pose blueNearStart = new Pose(19.5 + xOffset,120.5 + yOffset, Math.toRadians(270));
    private final Pose blueNearStart = new Pose(23 + xOffset, 140.5 - yOffset, Math.toRadians(270));
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
    private final Pose thirdBlueNearShoot = new Pose (50, 87, Math.toRadians(180));
    private final Pose finalShootAndPark = new Pose (58, 104, Math.toRadians(180));

    // these are red coords
    //private final Pose gateIntake = new Pose(129.14, 58.7, Math.toRadians(30));
    private final Pose gatePreIntake = new Pose(13.0, 58.7, Math.toRadians(140));
    private final Pose gateIntake = new Pose(GATE_X, GATE_Y, Math.toRadians(GATE_HEADING));
    private final Pose gateCloser = new Pose(12.1, 60, Math.toRadians(150));
    private final Pose gateIntakeBack = new Pose(12, 52, Math.toRadians(140));
    private final Pose gateIntakeBack2 = new Pose(12, 54, Math.toRadians(140));
    private final Pose gateControl = new Pose(30.0, 58.7, Math.toRadians(180));
    private final Pose gateCurve = new Pose (13, 53, Math.toRadians(90));

    private final Pose spikeStart1 = new Pose (42, 35.0, Math.toRadians(180));
    private final Pose spikeEnd1 = new Pose (10.4, 35.0, Math.toRadians(180));
    private final Pose spikeStart1Control = new Pose(57.4, 37.27, Math.toRadians(180));

    // next set of spikes is one tile away
    private final Pose spikeStart2 = new Pose (45, 35.0 + 23.5, Math.toRadians(180));
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
    private final Pose endParkFar = new Pose (33,yOffset + 3 , Math.toRadians(180));

    // human-player preloads (old way)
    /*
    private final Pose wallFar = new Pose(xOffset + 5, 23.6 + yOffset, Math.toRadians(210));
    private final Pose wallControl = new Pose(57.624, 23.6 + yOffset, Math.toRadians(270));
    private final Pose wallClose = new Pose(xOffset + 4, yOffset + 3, Math.toRadians(255));
    private final Pose wallClosish = new Pose(xOffset + 4, yOffset + 5, Math.toRadians(270));
    */

    // human-player preloads (try straight 90-degree routing)
    private final Pose wallFar = new Pose(xOffset, 23.6 + yOffset, Math.toRadians(270));
    //private final Pose wallControl = new Pose(57.624, 23.6 + yOffset, Math.toRadians(270));
    private final Pose wallClose = new Pose(xOffset, yOffset + 3, Math.toRadians(270));
    private final Pose wallClosish = new Pose(xOffset + 4, yOffset + 5, Math.toRadians(300));
    private final Pose gatePoint = new Pose(50, 35.0 + 23.5, Math.toRadians(180));

    private final Pose firstPoint = new Pose(65.7, 99.5, Math.toRadians(180));
    private final Pose firstSoloPoint = new Pose(41.5, 115.5, Math.toRadians(180));

    private final Pose middleShoot = new Pose (57.6, 74.6, Math.toRadians(180));
    // private final Pose humanPlayerIntake = new Pose(9.8, 8.6,Math.toRadians(180));
    private final Pose humanPlayerIntakeTwo = new Pose(11.42,23,Math.toRadians(180));
    private final Pose secretTunnelIntake = new Pose(11.42,35,Math.toRadians(130));
    private final Pose humanPlayerIntake = new Pose(19, 8.124, Math.toRadians(215));
    private final Pose theCorner = new Pose(yOffset, xOffset, Math.toRadians(180));
    private final Pose tiltedCorner = new Pose(9, xOffset, Math.toRadians(180));
    private final Pose slightlyBack = new Pose(12, xOffset, Math.toRadians(180));
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
    public FollowPathCommand curveBetween(Pose b, Pose c,Pose e,  double speed, boolean tangent){
        return curveBetween(b, c, e, speed, tangent, Constants.brakingStrength);
    }
    public FollowPathCommand curveBetween(Pose b, Pose c,Pose e,  double speed, boolean tangent, double brakingStrength) {
        Pose begin = convert(b);
        Pose end = convert(e);
        Pose control = convert(c);
        Path p = new Path(new BezierCurve(begin, control, end));

        if (tangent){
            p.setTangentHeadingInterpolation();
            p.reverseHeadingInterpolation();
        } else if (begin.getHeading() != end.getHeading()) {
            p.setLinearHeadingInterpolation(begin.getHeading(), end.getHeading());
        } else {
            p.setConstantHeadingInterpolation(end.getHeading());
        }
        p.setBrakingStrength(brakingStrength);  // same as passing in setGlobalDeceleration()
        return new FollowPathCommand(p, speed);
    }

    class FollowPathCommand extends CommandBase {
        Path path;
        double speed;
        double startTime;

        public FollowPathCommand(Path p, double s) {
            path = p;
            speed = s;
            addRequirements(drive);
        }
        public void initialize() {
            follower.followPath(path);
            follower.setMaxPower(speed);
            startTime = time;
        }
        public boolean isFinished() {
            double elapsed = time - startTime;
            return !follower.isBusy() || elapsed > 3;
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
            auto.addCommands(
                    new PrepareToShoot(),
                    new ParallelCommandGroup(
                            new AutoOuttake()
                    ),
                    new Delay(0.1)
            );
        }

        // collect and shoot human-player preloads
        auto.addCommands(
                new PrepareToShoot(),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        new SequentialCommandGroup(
                                pathBetween(blueFarStart, tiltedCorner, 0.5),
                                new Delay(0.5)
                        )
                ),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        new SequentialCommandGroup(
                                pathBetween(tiltedCorner, slightlyBack, 0.5),
                                new Delay(0.2)
                        )
                ),
                new ParallelCommandGroup(
                        //                        new SortSpindex(),
                        pathBetween(slightlyBack, blueFarShoot, 1.0)
                ),
                new AutoOuttake()
        );

        // collect and shoot audience spike mark
        auto.addCommands(
                pathBetween (blueFarShoot, spikeStart1, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart1, spikeEnd1, 0.4)
                ),
                //new PrepareToShoot(),
                //new SoftIntake(),
                //new Delay(1.0),
                new PrepareToShoot(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                //                                new SortSpindex(),
                                new AutoOuttake()
                        ),
                        pathBetween(spikeEnd1, blueFarShoot, 1.0)
                )
        );

        /* Old routing
        auto.addCommands(
                //curveBetween(blueFarShoot, wallControl, wallFar,  1.0),
                pathBetween(blueFarShoot, wallFar,  1.0),
                new PrepareToShoot(),
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
                new ParallelCommandGroup(
                        new SortSpindex(),
                        pathBetween(wallClose, blueFarShoot, 1.0)
                ),
                new AutoOuttake()
        );*/

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
        // gate scoop
        if (gatePickUp) {
            auto.addCommands(
                    new PrepareToShoot(),
                    new ParallelRaceGroup(
                            new SequentialCommandGroup(
                                    curveBetween(blueFarShoot, theCorner, secretTunnelIntake, 0.7, false),
                                    new Delay(1.0)
                            ),
                            new AutoIntake()
                    ),
                    new ParallelCommandGroup(
                            //                            new SortSpindex(),
                            pathBetween(secretTunnelIntake, blueFarShoot, 1.0)
                    ),
                    //new SoftIntake(),
                    new AutoOuttake()
            );

            //2nd gate scoop
            auto.addCommands(
                    new PrepareToShoot(),
                    new ParallelRaceGroup(
                            new SequentialCommandGroup(
                                    curveBetween(blueFarShoot, theCorner, secretTunnelIntake, 0.7, false),
                                    new Delay(1.0)
                            ),
                            new AutoIntake()
                    ),
                    new ParallelCommandGroup(
                            //                            new SortSpindex(),
                            pathBetween(secretTunnelIntake, blueFarShoot, 1.0)
                    ),
                    //new SoftIntake(),
                    new AutoOuttake()
            );
        } else {
            auto.addCommands(
                    new PrepareToShoot(),
                    new ParallelRaceGroup(
                            new AutoIntake(),
                            new SequentialCommandGroup(
                                    pathBetween(blueFarShoot, humanPlayerIntake, 0.6),
                                    new Delay(1.0)
                            )
                    ),
                    new SoftIntake(),
                    new ParallelCommandGroup(
                            //                                new SortSpindex(),
                            pathBetween(humanPlayerIntake, blueFarShoot, 1.0)
                    ),
                    new AutoOuttake()
            );
        }

        auto.addCommands(
                new PrepareToShoot(),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        new SequentialCommandGroup(
                                pathBetween(blueFarShoot, tiltedCorner, 0.55),
                                new Delay(2.0)
                        )
                )
        );

        auto.addCommands(
                new SpindexMode()
        );

        return auto;
    }


    //
    // near-zone auto pathing
    //

    private void doGateIntake(SequentialCommandGroup auto, Pose fromWhere, Pose toWhere) {
        auto.addCommands(
            new ParallelRaceGroup(
                new AutoIntake(),
                new SequentialCommandGroup(
                    curveBetween(fromWhere, gatePoint, gateIntake, 0.8, false, 0.6),
                    new Delay(1.0),
                    pathBetween(gateIntake, gateIntakeBack, 0.4),
                    new Delay(0.67)
                )
            ),
            new ParallelCommandGroup(
                new AutoOuttake(),
                curveBetween(gateIntakeBack, gatePoint, toWhere, 1.0, false, 0.6)
            )
        );
    }

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
                    new ParallelCommandGroup(
                            //pathBetween(blueNearStart, thirdBlueNearShoot, 1.0),
                            curveBetween(blueNearStart,firstPoint,spikeStart2, 1.0, false, 0.6),
                            new AutoOuttake()

                    ),
                    new Delay(0.100)
            );
            spikeStart = spikeStart2;
        } else {
            spikeStart = blueNearStart;
        }

        // pick up and shoot middle spike mark
        auto.addCommands(
                         new ParallelRaceGroup(
                                               new AutoIntake(),
                                               new SequentialCommandGroup(
                                                                          pathBetween(spikeStart, spikeEnd2, 0.56),
                                                                          new Delay(0.2)
                                                                          )
                                               )
                         );
        auto.addCommands(
                         new PrepareToShoot(),
                         new ParallelCommandGroup(
                                                  new AutoOuttake(),
                                                  pathBetween(spikeEnd2, middleShoot, 1.0)
                                                  )
                         );

        doGateIntake(auto, middleShoot, middleShoot);

        //gate intake 2
        if (gateIntake2) {
            doGateIntake(auto, middleShoot, middleShoot);
        }

        // intake spike 3 (further from audience)
        auto.addCommands(
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                                   new AutoIntake(),
                                                   new PrepareToShoot()
                                                   ),
                        new SequentialCommandGroup(
                                                   pathBetween(middleShoot, spikeStart3, 0.85),
                                                   pathBetween(spikeStart3, spikeEnd3, 0.67),
                                                   pathBetween(spikeEnd3, finalShootAndPark, 1.0)
                                                   )
                ),
                new AutoOuttake()
        );

        auto.addCommands(
                new SpindexMode()
        );

        return auto;
    }

    private Command soloPathing() { //if alliance partner cannot pick up spikemarks
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
                    new ParallelCommandGroup(
                            //pathBetween(blueNearStart, thirdBlueNearShoot, 1.0),
                            curveBetween(blueNearStart,firstSoloPoint,spikeStart3, 1.0, false),
                            new AutoOuttake()
                    ),
                    new Delay(0.100)
            );
        } else {
            spikeStart = blueNearStart;
        }

        // pick up and shoot middle spike mark
        auto.addCommands(
                // pathBetween(thirdBlueNearShoot, spikeStart2, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        new SequentialCommandGroup(
                                pathBetween(spikeStart3, spikeEnd3, 0.3)
                        )
                )
                //  new SoftIntake()
        );

        auto.addCommands(
                new PrepareToShoot(),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new Delay(1.2),
                                new AutoOuttake()
                        ),
                        pathBetween(spikeEnd3, thirdBlueNearShoot, 1.0)
                )

        );

        auto.addCommands(
                pathBetween(thirdBlueNearShoot, spikeStart2, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart2, spikeEnd2, 0.3)
                ),
                //pathBetween(spikeEnd2, gatePreIntake, 1.0),
                //pathBetween (gatePreIntake, gateIntake, 1.0),
                new ParallelCommandGroup(
                        new AutoOuttake(),
                        curveBetween(spikeEnd2, gatePoint, thirdBlueNearShoot, 1.0, true)
                )
        );

        // intake spike 3 (further from audience)
        auto.addCommands(
                pathBetween(thirdBlueNearShoot, spikeStart1, 1.0),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        pathBetween(spikeStart1, spikeEnd1, 0.3)
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                //                                new SortSpindex(),
                                new AutoOuttake()
                        ),
                        curveBetween(spikeEnd1, gatePoint, thirdBlueNearShoot, 1.0, false)
                )
        );

        auto.addCommands(
                curveBetween(thirdBlueNearShoot, gatePoint, gatePreIntake, 1.0, false),
                new ParallelRaceGroup(
                        new AutoIntake(),
                        new SequentialCommandGroup(
                                pathBetween(gatePreIntake, gateIntake, 0.7),
                                new Delay(1.0),
                                pathBetween(gateIntake, gateIntakeBack, 0.35)
                        )
                ),
                //  new SoftIntake(),
                new ParallelCommandGroup(
                        //  new PrepareToShoot(),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        //                                        new SortSpindex(),
                                        new AutoOuttake()
                                ),
                                curveBetween(gateIntakeBack, gatePoint, finalShootAndPark, 1.0, false)
                                //pathBetween(gateIntakeBack, thirdBlueNearShoot, 1.0)

                        )
                )

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
        if (operator.wasJustPressed(GamepadKeys.Button.A)){
            soloNear = true;
        }
        // if (whenOpenGate == 0) openDescription = "Never";
        // if (whenOpenGate == 1) openDescription = "After Spike3 pickup";


        telemetry.addData("Preloads (Dpad Up to toggle)", usePreloads);
        //  telemetry.addData("Open Gate (Dpad Down to toggle)", openDescription);
        // telemetry.addData("Pick up third spike (Dpad Left to toggle", audienceSpike);
        telemetry.addData("Intake from gate far zone (Dpad Left to toggle)", gatePickUp);
        telemetry.addData("Near zone open gate (Dpad Right to toggle)", closeGateOpen);
        telemetry.addData("Near zone 2nd gate intake (Dpad Down to toggle)", gateIntake2);
        telemetry.addData("Solo auto for near (A to toggle)", soloNear);
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
                if (soloNear) {
                    cmds = soloPathing();
                }
                else{
                    cmds = nearPathing();
                }
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
        super.loop();
        follower.update();
        // the command-scheduler is run in our super-class
    }

    @Override
    public void stop() {
        drive.read_sensors(time);
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putFloat("heading", (float)drive.getPosition().getHeading(AngleUnit.DEGREES));
        editor.putFloat("x", (float)drive.getPosition().getX(DistanceUnit.INCH));
        editor.putFloat("y", (float)drive.getPosition().getY(DistanceUnit.INCH));
        editor.putFloat("turret", (float)turret.servoReset);
        editor.putInt("spindex-target", spindexer.targetAngle);
        editor.putInt("obelisk", turret.pattern);
        editor.apply();

        super.stop();
    }
}
