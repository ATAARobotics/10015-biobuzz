
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test")
public class ShooterTestBiobuzz extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor motor = hardwareMap.get(DcMotor.class, "shooterL");

        double motorPower = 0.0;

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        waitForStart();

        while (opModeIsActive()) {

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

            motor.setPower(motorPower);

            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.update();

            // Remember current button states for next loop
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
        }

        motor.setPower(0);
    }
}