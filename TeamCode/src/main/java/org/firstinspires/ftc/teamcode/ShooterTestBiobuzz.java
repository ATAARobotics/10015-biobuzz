

package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;


import java.util.LinkedList;

@Configurable
@TeleOp(name = "Motor Test")
public class ShooterTestBiobuzz extends OpMode {
    public static int dataPoints = 30;
    class VelocityData {
        public double time;
        public double velocity;
        public VelocityData(double t, double v) {
            time = t;
            velocity = v;


        }
    }

    public static double kP = 20;
    public static double kV = 0.7;
    public static double AVERAGE_EXPONENT = 1.2;
    public static double motorPower = 0.0;
    LinkedList<VelocityData> recentVelocity = null;
    MotorEx motor;
    MotorEx motor2;
  //  double motorPower = 0.0;
    double targetRpm = 0.0;
    double rps;
    int ticks;
    int lastTicks;
    double lastTime;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    HyperTelemetry telem;

    @Override
    public void init() {
        motor = new MotorEx(hardwareMap,"shooterL");
        motor2 = new MotorEx(hardwareMap, "encoder");
	telem = new HyperTelemetry(telemetry);
    recentVelocity = new LinkedList<VelocityData>();
  //  motor.setRunMode(Motor.RunMode.VelocityControl);
      //  motor.setVeloCoefficients(kP, 0, 0);
       // motor.setFeedforwardCoefficients(0, kV);


    }
    
    @Override
    public void loop() {

        double v = motor.getVelocity()/28*60; //this gives us rpm
        recentVelocity.addLast(new VelocityData(time, v));
    while(recentVelocity.size() > dataPoints) {
        recentVelocity.removeFirst();
    }
        double total = 0.0;
        double amount = 1;
        double totalAmount = 0.0;
        for(VelocityData data:recentVelocity){
            totalAmount = totalAmount + amount;
            total = total + (data.velocity * amount); // adding recentvelocity data
            amount *= AVERAGE_EXPONENT; //1.0 is equal to normal average, more than that
            //creates an exponential graph
        }
        double averageVelocity = total / totalAmount; //finding average - total value of data/number of data points
        telem.log("Average-RPM", averageVelocity);
	/*    recentRpm.addLast(new RpmData(time, ticks)); //add a new data point (recent)
	while(recentRpm.size() > 5){
	    recentRpm.removeFirst();
	}

        if(recentRpm.size() >= 2) { // 2 or more data points
            // recentRpm.removeFirst();
            double previousTime = recentRpm.get(recentRpm.size() - 2).time; //ex. 3 data points.
            // when we use index, it counts the size as 0, 1, 2. 3-1 is giving current data.
            //3-2 is previous data.
            double previousTicks = recentRpm.get(recentRpm.size() - 2).ticks;
            double rps = ((ticks-previousTicks)/28.0)/(time-previousTime);
            double rpm = rps * 60.0;
            telem.log("rpm-0", rpm);
        }

        if(recentRpm.size() >= 2) { // 2 or more data points
            // recentRpm.removeFirst();
            double previousTime = recentRpm.get(0).time; //'get' uses an index,
            // so it counts the data points starting from 0. 0 would be the
            //first data point
            double previousTicks = recentRpm.get(0).ticks;
            double rps = ((ticks-previousTicks)/28.0)/(time-previousTime);
            double rpm = rps * 60.0;
            telem.log("rpm-1", rpm);
        }
*/
	telem.log("time", time);
	telem.log("ticks", ticks);
    telem.log("RPM", v);
    telem.log("targetRpm", targetRpm);


        // Increase power by 0.1 on each new D-pad up press
        if (gamepad1.dpad_up && !lastDpadUp) {
           // motorPower += 0.1;
            targetRpm += 100;
        }

        // Decrease power by 0.1 on each new D-pad down press
        if (gamepad1.dpad_down && !lastDpadDown) {
            //motorPower -= 0.1;
            targetRpm -= 100;
        }

        targetRpm = Math.max(0.0, Math.min(3000.0, targetRpm));

/*      //bangbang controller
        if (v < targetRpm){
            motorPower = 1.0;

        } else {
            motorPower = 0.0;
        }

 */
        // Clamp power between 0.0 and 1.0
        motorPower = Math.max(0.0, Math.min(1.0, motorPower));
        motor.set(motorPower);

        // Remember current button states for next loop
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastTicks = ticks;
        lastTime = time;

	// update panels etc
	telem.update();
    }

}

