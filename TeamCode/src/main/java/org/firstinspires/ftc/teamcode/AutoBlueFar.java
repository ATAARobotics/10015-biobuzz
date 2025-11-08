package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoBlueFar", group="Opmode")
public class AutoBlueFar extends Auto
{
    public Alliance getAlliance() { return Alliance.BLUE; }
    public AutoStartPos getStartPos() {return AutoStartPos.FAR;}
}
