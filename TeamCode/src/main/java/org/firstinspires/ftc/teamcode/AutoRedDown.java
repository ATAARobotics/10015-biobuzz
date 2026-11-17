package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoRedDown", group="Opmode")
public class AutoRedDown extends Auto
{
    public Alliance getAlliance() { return Alliance.RED; }
    public StartZone getStartZone() { return StartZone.DOWN; }
}