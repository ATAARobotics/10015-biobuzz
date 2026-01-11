package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
@Autonomous(name = "MoveTo")
@Configurable
public class MoveTo extends OpMode   {
    public Drive drive;

    public static double TARGET_X = 0;
    public static double TARGET_Y = -1.612;
    public static double TARGET_H = 0;

    Drive.MoveTo moveToTarget;

    // x, y are in CM, heading is degrees

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(drive);

        drive.setPosition(new SparkFunOTOS.Pose2D(0,-1.612,0));
        moveToTarget = drive.moveTo(TARGET_X, TARGET_Y, TARGET_H, 0.5, 0.02);
        CommandScheduler.getInstance().schedule(moveToTarget);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        moveToTarget.setTarget(TARGET_X,TARGET_Y,TARGET_H);
        drive.read_sensors();

        CommandScheduler.getInstance().run();

        TelemetryPacket pack = new TelemetryPacket();
        drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        // note, seems that "drawing stuff" commands have to go in their own packet
    }

    @Override
    public void stop() {
        drive.stop();
        CommandScheduler.getInstance().reset();
    }
}
*/
