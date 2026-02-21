package org.firstinspires.ftc.teamcode.NextFTC;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
@Configurable
public class Flywheel implements Subsystem {
    public static final Flywheel INSTANCE = new Flywheel();
    public static final double FlywheelVelo = 2600;

    public boolean flywheelOn;

    private Flywheel() { }

    private final MotorEx Shooter1 = new MotorEx("Shooter1");
    private final MotorEx Shooter2 = new MotorEx("Shooter2").reversed();
    private final MotorEx Turret = new MotorEx("Turret").brakeMode();
    private final CRServoEx Gate = new CRServoEx("Gate");
    private final ServoEx Hood = new ServoEx("Hood");

    public static double openPos = 0;
    public static double closePos = 0.6;

    public static double p = 0.005; // CONFIGURE

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(p, 0, 0)
            .basicFF(0.01, 0.02, 0.03)
            .build();


    public void On(){
        flywheelOn = true;
        controller.setGoal(new KineticState(0,FlywheelVelo));
    }

    public void Off(){
        flywheelOn = false;
        controller.setGoal(new KineticState(0,0));
    }

    /* public final Command off = new RunToVelocity(controller, 0.0);
    public final Command on = new RunToVelocity(controller, FlywheelVelo);*/


    public void openGate(){
        Gate.setPower(-1);
    }

    public void closeGate(){
        Gate.setPower(1);
    }

    private ElapsedTime timer;

    @Override
    public void initialize(){
        timer = new ElapsedTime();
    }

    @Override
    public void periodic() {
        Shooter1.setPower(controller.calculate(Shooter1.getState()));
        Shooter2.setPower(controller.calculate(Shooter1.getState()));
    }
}