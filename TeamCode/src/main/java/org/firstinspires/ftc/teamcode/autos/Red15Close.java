package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.Optimised15Ball;

@Autonomous(name = "RED 15 Ball Close Zone", group = "15 Ball Close Zone")
public class Red15Close extends Optimised15Ball {
    public Red15Close(){
        super(true);
    }
}
