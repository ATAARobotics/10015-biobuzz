package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// this wraps up the various methods of telemetry and locations where
// things go.
//
// this means we can do "nice" things like log stuff to files during
// matches
public class HyperTelemetry {

    JoinedTelemetry telemetry;
    TelemetryPacket pack;

    public HyperTelemetry(Telemetry driver_station) {//, TelemetryPacket dashboard) {
        PanelsTelemetry pt = PanelsTelemetry.INSTANCE;
        this.telemetry = new JoinedTelemetry(pt.getFtcTelemetry(), driver_station);
//        this.pack = dashboard;
    }

    // log to the driver's station (only)
    public void logDrivers(String key, Object raw_format, Object... args) {
        String format = raw_format.toString();
        telemetry.addData(key, format, args);
    }

    public void log(String key, Object value) {
        PanelsTelemetry.INSTANCE.getTelemetry().addData(key, value);
        //pack.put(key, value);
        //RobotLog.ii("HyperDroid", "\"%s\", \"%s\"", key, value);
    }

    public void logBoth(String key, Object value) {
        log(key, value);
        logDrivers(key, value);
    }

    public void update() {
        telemetry.update();
    }
}
