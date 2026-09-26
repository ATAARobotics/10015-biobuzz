

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


import java.util.LinkedList;


@TeleOp(name = "Motor Test")
public class ShooterTestBiobuzz extends OpMode {


    class RpmData{
        public double time;
        public double rpm;
        public RpmData(double t,double r){
            time=t;
            rpm=r;
        }
    }
    LinkedList<RpmData> recentRpm;
    MotorEx motor;
    double motorPower = 0.0;
    double rpm;
    int ticks;
    int lastTicks;
    double lastTime;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;


    @Override
    public void init() {
        motor = new MotorEx(hardwareMap,"shooterL");
    }
    @Override
    public void loop() {
        ticks = motor.getCurrentPosition();
        double deltaTime = time - lastTime;
        double deltaTicks = ticks - lastTicks;
        if (lastTicks != 0){
            // There's 28 ticks per revolution on our motor
            // so rps is revs per second
            double rps = (deltaTicks/28.0)/deltaTime;
            rpm = rps*60.0;


            recentRpm.addLast(new RpmData(time,rpm));
            while(recentRpm.size()>5){
                recentRpm.removeFirst();
            }
        }


        // Increase power by 0.1 on each new D-pad up press
        if (gamepad1.dpad_up && !lastDpadUp) {
            motorPower += 0.1;
        }


        // Decrease power by 0.1 on each new D-pad down press
        if (gamepad1.dpad_down && !lastDpadDown) {
            motorPower -= 0.1;
        }


        // Clamp power between 0.0 and 1.0
        motorPower = Math.max(0.0, Math.min(1.0, motorPower));


        motor.set(motorPower);


        logTelemetry();
        // Remember current button states for next loop
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastTicks = ticks;
        lastTime = time;
    }
    protected void logTelemetry() {
        //TelemetryPacket pack = new TelemetryPacket();
        HyperTelemetry telem = new HyperTelemetry(telemetry);
        telem.logBoth("rpm", rpm);
        telem.logBoth("power", motorPower);
        telem.update();
    }
    @Override
    public void stop() {
        motor.set(0);
    }
}

