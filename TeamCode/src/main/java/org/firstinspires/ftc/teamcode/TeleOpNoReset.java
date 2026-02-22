package org.firstinspires.ftc.teamcode;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public abstract class TeleOpNoReset extends TeleOp {

    @Override
    public void start() {
        runtime.reset();
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        float x = prefs.getFloat("x", 0.0f);
        float y = prefs.getFloat("y", 0.0f);
        float h = prefs.getFloat("heading", 0.0f);
        drive.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, h));
        int spinTarget = prefs.getInt("spindex-target", 0);
        float turretTicks = prefs.getFloat("turret", 0.0f);
        turret.lastEncoder = turretTicks;
        int pattern = prefs.getInt("oblelisk", -1);
        turret.pattern = pattern;

        // TODO: this isn't working, it seems like the encoder isn't
        // being reset to 0 but is changing somehow when we start
        // teleop? So we just reset and hope it was in a correct
        // position (otherwise, operator has to fix it)
        ////spindexer.targetAngle = spinTarget;
        spindexer.reset();
    }

}
