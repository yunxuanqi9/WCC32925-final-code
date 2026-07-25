package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autos.oldIgnore.closeZone12;

@Disabled
@Autonomous(name = "RED 12 Ball Close Zone", group = "12 Ball Close Zone")
public class Red12Close extends closeZone12 {
    public Red12Close(){
        super(true);
    }
}
