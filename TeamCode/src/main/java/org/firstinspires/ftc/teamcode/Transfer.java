package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo    ;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Transfer extends SubsystemBase {
    MotorEx intakeMotor;
    MotorEx transferMotor;
    Servo pollenBlocker;
    Servo magazineServo;
    Servo crashBarServo1;
    Servo crashBarServo2;
    Servo nectarBlocker;
    public Transfer(HardwareMap hardwareMap, HyperTelemetry telem){
        intakeMotor = new MotorEx(hardwareMap, "intake");
        transferMotor = new MotorEx(hardwareMap, "transfer");
       // magazineServo = hardwareMap.get(Servo.class, "magazine");
        pollenBlocker = hardwareMap.get(Servo.class, "pollenBlocker");
        //nectarBlocker = hardwareMap.get(Servo.class, "nectarBlocker");
       // crashBarServo1 = hardwareMap.get(Servo.class, "crashBar1");
        //crashBarServo2 = hardwareMap.get(Servo.class, "crashBar2");
    }

    public void runIntakeMotor(){

    }
}
