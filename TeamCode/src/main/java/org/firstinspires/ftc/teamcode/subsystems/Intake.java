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

    public static double ballWait = 0.1;

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


    public Command preCheckBalls = new InstantCommand(() -> {
        if(currReading > twoBallCurrent){
            Balls = 2;
        }
        else if(currReading > threeBallCurrent){
            Balls = 3;
        }
        else{
            Balls = 0;
        }
    });

    public Command checkBalls = new InstantCommand(() -> {
        currReading = intakeMotor.getMotor().getCurrent(CurrentUnit.AMPS);
        if(currReading > twoBallCurrent && Balls == 2){
            RGB.setPosition(0.7);
        }
        else if(currReading > threeBallCurrent && Balls == 2){
            RGB.setPosition(0.6);
        }
        else{
            RGB.setPosition(0.4);
        }
    });

    public Command detectBalls = new SequentialGroup(
            preCheckBalls.afterTime(0.1).thenWait(ballWait),
            checkBalls

    );


    public Command Unclog = new SetPower(intakeMotor, -1);

    public Command Nudge = new SetPower(intakeMotor, -1);

    public Command On = new SetPower(intakeMotor,1);


    public Command Off = new SetPower(intakeMotor, 0);

    public Command intakeArtifacts =
            new ParallelGroup(
                    On,
                    detectBalls
            );

    public void On() {
        intakeMotor.setPower(1);
    }

    @Override
    public void periodic(){
        ActiveOpMode.telemetry().addData("Intake current",currReading);
    }
}
