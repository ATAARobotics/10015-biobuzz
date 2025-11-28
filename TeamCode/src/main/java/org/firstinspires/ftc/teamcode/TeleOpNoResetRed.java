package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp NORESET (RED)", group="Opmode")
public class TeleOpNoResetRed extends TeleOp {
    public Alliance getAlliance() { return Alliance.RED; }

    @Override
    public void start() {
        runtime.reset();
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        float x = prefs.getFloat("x", 0.0f);
        float y = prefs.getFloat("y", 0.0f);
        float h = prefs.getFloat("heading", 0.0f);
        drive.setPosition(new Pose2D(DistanceUnit.METER, x, y, AngleUnit.DEGREES, h));
        //float t = prefs.getFloat("turret", 0.0f);
        //turret.setAngleReset(t);
        turret.angleReset();
    }
}
