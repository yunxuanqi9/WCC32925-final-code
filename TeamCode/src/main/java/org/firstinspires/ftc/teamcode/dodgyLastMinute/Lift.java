package org.firstinspires.ftc.teamcode.dodgyLastMinute;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * The lift subsystem implemented from
 * <a href="https://nextftc.dev/guide/subsystems/lift">docs</a>.
 */
public class Lift implements Subsystem {


    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    private final MotorEx motor = new MotorEx("Turret");

    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .elevatorFF(0)
            .build();

    public Command toLow = new RunToPosition(controlSystem, 0).requires(this);
    public Command toMiddle = new RunToPosition(controlSystem, 500).requires(this);
    //public Command toHigh = new RunToPosition(controlSystem, 1200).requires(this);


    public void toHigh(){
        controlSystem.setGoal(new KineticState(500));
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}