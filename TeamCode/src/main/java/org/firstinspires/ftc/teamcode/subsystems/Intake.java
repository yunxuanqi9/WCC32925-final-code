package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    private Intake(){}
    public static final Intake INSTANCE = new Intake();
    private final MotorEx intakeMotor = new MotorEx("Intake").reversed();

    boolean correctDirection = true;

    public Command On = new SetPower(intakeMotor,1);

    public Command Off = new SetPower(intakeMotor, 0);

    public Command Unclog = new SetPower(intakeMotor, -1);

    public void On() {
        intakeMotor.setPower(1);
    }
}
