package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter Test")
public class ShooterTestBiobuzz extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor motor = hardwareMap.get(DcMotor.class, "shooterL");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(1.0);   // 100% power
        }

        motor.setPower(0);
    }
}

