package org.firstinspires.ftc.teamcode.NextFTC;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    private Intake(){}
    public static final Intake INSTANCE = new Intake();
    private final MotorEx intakeMotor = new MotorEx("Intake").reversed();
    public void On(){
        intakeMotor.setPower(1);
    }
    public void Off(){
        intakeMotor.setPower(0);
    }
}
