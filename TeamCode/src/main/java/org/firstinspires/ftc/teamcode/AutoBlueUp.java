package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlueUp", group="Opmode")
public class AutoBlueUp extends Auto
{
    public Alliance getAlliance() { return Alliance.BLUE; }
    public StartZone getStartZone() { return StartZone.UP; }
}