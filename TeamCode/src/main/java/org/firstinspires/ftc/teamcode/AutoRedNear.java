package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoRedNear", group="Opmode")
public class AutoRedNear extends Auto
{
    public Alliance getAlliance() { return Alliance.RED; }
    public AutoStartPos getStartPos() {return AutoStartPos.NEAR;}
}
