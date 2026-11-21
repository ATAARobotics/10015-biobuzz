package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Reset Storage")
public class ResetStorage extends OpMode {
    @Override
    public void init(){
        OpmodeStorage.reset();
    }
    @Override
    public void loop(){

    }
}
