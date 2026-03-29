package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    public static double threeBallCurrent = 4;

    private Intake(){}
    public static final Intake INSTANCE = new Intake();
    private final MotorEx intakeMotor = new MotorEx("Intake").reversed();

    boolean correctDirection = true;

    public double currReading;

    public Command On = new SetPower(intakeMotor,1);

    public Command Off = new SetPower(intakeMotor, 0);

    public Command Unclog = new SetPower(intakeMotor, -1);

    public Command Nudge = new SetPower(intakeMotor, -0.4);

    public Command intakeArtifacts =
            new SequentialGroup(
                    On,
                    new WaitUntil(() -> currReading >= threeBallCurrent),
                    Off
            );

    public void On() {
        intakeMotor.setPower(1);
    }

    @Override
    public void periodic(){
        currReading = intakeMotor.getMotor().getCurrent(CurrentUnit.AMPS);

        ActiveOpMode.telemetry().addData("Intake current",currReading);
    }
}
