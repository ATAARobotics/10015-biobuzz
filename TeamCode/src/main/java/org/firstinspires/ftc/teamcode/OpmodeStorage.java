package org.firstinspires.ftc.teamcode;
//Make sure this matches your file location
import com.pedropathing.math.Pose;
public class OpmodeStorage {
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    // default pose to avoid null errors
    public static boolean autoRan = false;
}