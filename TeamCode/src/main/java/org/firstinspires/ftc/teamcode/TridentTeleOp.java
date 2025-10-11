package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Trident TeleOp", group="Opmode")
public class TridentTeleOp extends OpMode {
    GamepadEx driver;
    GamepadEx operator;
    Shooter shooter;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        // read controls and sensors
        driver.readButtons();
        operator.readButtons();
        shooter.read_sensors(time);

        // decide what we're going to do
        shooter.loop(driver);

        // send telemetry etc
        //telemetry.addData("Battery Voltage", battery.getVoltage());
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        //pack.put("battery", battery.getVoltage());
        shooter.add_telemetry(pack, telemetry);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
    }

}
