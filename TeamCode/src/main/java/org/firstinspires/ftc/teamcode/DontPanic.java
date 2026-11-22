package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Don't Panic")
public class DontPanic extends OpMode {
    @Override
    public void init(){
        OpmodeStorage.panicMode = false;
    }
    @Override
    public void loop(){

    }
}
