package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoRedUp", group="Opmode")
public class AutoRedUp extends Auto
{
    public Alliance getAlliance() { return Alliance.RED; }
    public StartZone getStartZone() { return StartZone.UP; }
}
