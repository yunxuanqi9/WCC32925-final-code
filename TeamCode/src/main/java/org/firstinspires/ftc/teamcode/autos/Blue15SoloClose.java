package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.Optimised15Ball;

@Autonomous(name = "BLUE 15 Ball Solo Close Zone", group = "15 Ball Close Zone")
public class Blue15SoloClose extends Optimised15Ball {
    public Blue15SoloClose(){
        super(false, true);
    }
}
