package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.CornerIntake21;

@Autonomous(name = "RED 21 Ball Corner Intake", group = "21 Ball Far Zone")
public class Red21Far extends CornerIntake21 {
    public Red21Far(){
        super(true);
    }
}
