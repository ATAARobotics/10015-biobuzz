package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake extends SubsystemBase {
   public MotorEx intake;
    public Intake (HardwareMap hardwareMap){
        intake = new MotorEx(hardwareMap, "intake");
    }
}