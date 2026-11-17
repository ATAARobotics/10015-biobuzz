package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlueDown", group="Opmode")
public class AutoBlueDown extends Auto
{
    public Alliance getAlliance() { return Alliance.BLUE; }
    public StartZone getStartZone() { return StartZone.DOWN; }
}