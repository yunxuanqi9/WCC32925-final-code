package org.firstinspires.ftc.teamcode.NextFTC;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() { }

    private final MotorEx shooter1 = new MotorEx("Shooter1");
    private final MotorEx shooter2 = new MotorEx("Shooter2");

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();

    public final Command off = new RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff");
    public final Command on = new RunToVelocity(controller, 500.0).requires(this).named("FlywheelOn");

    @Override
    public void periodic() {
        shooter1.setPower(controller.calculate(shooter1.getState()));
        shooter2.setPower(controller.calculate(shooter1.getState()));
    }
}