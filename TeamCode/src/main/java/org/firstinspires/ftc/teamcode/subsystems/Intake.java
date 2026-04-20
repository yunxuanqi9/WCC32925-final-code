package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Intake implements Subsystem {

    public static double nudgeTime = 0.6;

    public static double threeBallCurrent = 8;
    public static double twoBallCurrent = 4.2;

    private int Balls = 0;

    public static double ballWait = 0.03;

    private Intake(){}
    public static final Intake INSTANCE = new Intake();
    private final MotorEx intakeMotor = new MotorEx("Intake").reversed();

    public static double colour = 0.4;

    private double red = 0.277;
    private double yellow = 0.388;
    private double green = 0.566;

    public ServoEx RGB = new ServoEx("LED");

    boolean correctDirection = true;

    public double currReading;



    public Command Unclog = new SetPower(intakeMotor, -1);

    public Command Nudge = new SetPower(intakeMotor, -1);

    public Command On = new SetPower(intakeMotor,1);

    public Command slowFire = new SetPower(intakeMotor,0.7);

    public Command Off = new SetPower(intakeMotor, 0);


    public Command checkBalls = new InstantCommand(() -> {
        if(intakeMotor.getMotor().isOverCurrent()){
            RGB.setPosition(0.7);
        }
        else{
            RGB.setPosition(0.4);
        }
    }).afterTime(0.02);


    public Command intakeArtifacts =
            new ParallelGroup(
                    On,
                    checkBalls  
            );

    public void initialize(){
        intakeMotor.getMotor().setCurrentAlert(8,CurrentUnit.AMPS);
    }

    @Override
    public void periodic(){
        ActiveOpMode.telemetry().addData("Intake current",currReading);
    }
}
