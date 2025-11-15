package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret extends SubsystemBase {
    private Servo servo1;
    private Servo servo2;

    public void init() {
        servo1 = hardwareMap.get(Servo.class, "turretServo1");
        servo2 = hardwareMap.get(Servo.class, "turretServo2");
    }

    void faceToFieldAngle(double angle) {
        //servo1.setPosition(Drive.current_position.getHeading());
        //servo2.setPosition();
    }
}