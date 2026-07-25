package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autos.paths.farZoneSpike;

@Autonomous(name = "BLUE 21 Ball Far Zone - Spike", group = "21 Ball Far Zone")
public class Blue21NoGate extends farZoneSpike {
    public Blue21NoGate(){
        super(false, false);
    }
}
