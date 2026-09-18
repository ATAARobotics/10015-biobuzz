package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;


// this implements the suggested GoBilda fix for the Floodgate power
// issue .. that is to not change "too fast"
public class FloodMotor extends Motor {
    final double SLEW_RATE = 0.2;

    public FloodMotor(HardwareMap hw, String name){//, Motor.GoBILDA kind) {
        super(hw, name);//, kind);
    }

    @Override
    public void set(double power) {
        double currentPower = super.get();
        double delta = power - currentPower;
        double limited = Math.max(-SLEW_RATE, Math.min(delta, SLEW_RATE));
        super.set(currentPower += limited);
    }
}
