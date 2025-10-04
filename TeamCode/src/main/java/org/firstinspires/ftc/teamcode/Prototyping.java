package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Prototyping", group="Opmode")
public class  Prototyping extends OpMode {
    GamepadEx control;
    MotorEx motor0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double BIG_STEP_RPM = 250;
    private static final double SMALL_STEP_RPM = 10;
    VoltageSensor battery;
    double MAX_RPM = 5250;
    double rpmTarget;
    double currentRpm;

    PIDController velocity;
    public static double velocity_p = 0.01;
    public static double velocity_i = 0.0;
    public static double velocity_d = 0.0004;
    public static double kv = 0.0021; //kv is Feed Forward Model slope
    public static double ks = 1.4117; //ks is Feed Forward Model Y intercept (represents power needed to overcome friction)


    @Override
    //setting up the gamepad and motor
    public void init() {
        rpmTarget = 0;
        control = new GamepadEx(gamepad2);
        motor0 = new MotorEx(hardwareMap, "motor0");
        motor0.setRunMode(Motor.RunMode.RawPower);
        motor0. setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        velocity = new PIDController(velocity_p, velocity_i, velocity_d);
        battery = hardwareMap.voltageSensor.get("Control Hub");
    }

    @Override
    public void loop() {
        velocity.setP(velocity_p);
        velocity.setI(velocity_i);
        velocity.setD(velocity_d);
        velocity.setSetPoint(rpmTarget);
        double ticksPerSecond;
        ticksPerSecond = motor0.getVelocity();
        double power;
        currentRpm = (ticksPerSecond*60)/TICKS_PER_REV;
        double appliedVoltage;// = battery.getVoltage() * power;
        appliedVoltage = kv*rpmTarget+ks;
        power = appliedVoltage/battery.getVoltage();
        power += velocity.calculate(currentRpm); //Change power to += when Feed Forward is used
        if (rpmTarget == 0) power = 0;
        if(power < 0) power = 0;
        motor0.set(power); //when you move joystick, motor power changes
        telemetry.addData("motor0", power); //what you see on the screen
        telemetry.addData("rpmTarget", rpmTarget);
        telemetry.addData("Current RPM", currentRpm);
        //telemetry.addData("Applied Voltage", appliedVoltage);
        telemetry.addData("Battery Voltage", battery.getVoltage());

        // 3000 targetRPM for far, 1800 for close shot. (This was before we fixed the RPM target being the same as the RPM. We need to test this agian)

        control.readButtons();

        if (control.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            rpmTarget += BIG_STEP_RPM;
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            rpmTarget -= BIG_STEP_RPM;
            if(rpmTarget < 0) rpmTarget = 0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.Y)){
            rpmTarget += SMALL_STEP_RPM;
            if(rpmTarget > MAX_RPM) rpmTarget = MAX_RPM;
        }
        if (control.wasJustPressed(GamepadKeys.Button.A)) {
            rpmTarget -= SMALL_STEP_RPM;
            if (rpmTarget < 0) rpmTarget = 0;
        }
        if (control.wasJustPressed(GamepadKeys.Button.X)){
            rpmTarget = 0;
        }
        TelemetryPacket pack = new TelemetryPacket();
        pack.put("time", time);
        pack.put("ticksPerSecond", ticksPerSecond);
        pack.put("rpmTarget", rpmTarget);
        pack.put("Current RPM", currentRpm);
        pack.put("Power", power);
        pack.put("velocity_p:", velocity_p);
        pack.put("velocity_i:", velocity_i);
        pack.put("velocity_d:", velocity_d);
        //drive.add_telemetry(pack);
        FtcDashboard.getInstance().sendTelemetryPacket(pack);
        telemetry.update();
    }
}
