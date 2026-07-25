package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.Optimised15Ball;
import org.firstinspires.ftc.teamcode.autos.paths.Red12BallPath;

@Autonomous(name = "RED 12 Ball Solo Close Zone", group = "15 Ball Close Zone")

public class Red15SoloClose extends Optimised15Ball {
    public Red15SoloClose(){
        super(true, true);
    }
}