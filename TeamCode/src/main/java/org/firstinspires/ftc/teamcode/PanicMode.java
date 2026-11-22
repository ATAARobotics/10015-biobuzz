package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Panic Mode!!!")
public class PanicMode extends OpMode {
    @Override
    public void init(){
        OpmodeStorage.reset();
        OpmodeStorage.panicMode = true;
    }
    @Override
    public void loop(){

    }
}
