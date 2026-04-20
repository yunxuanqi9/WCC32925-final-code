package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.robotConstants.mainConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
@Configurable
public class Shooter implements Subsystem {

    public static double fixedSpeed;

    public static double setHood = 0; // CHANGE IN PANELS
    public static double maxHoodPose = 0.85;

    public static double kickerOn = 0.8;
    public static double kickerOff = 0.35;
    public static double kickerWait = 0.5;


    double PPR = 145.1; // 1150 motor
    double TURRET_LIMIT = 180;

    boolean veloLock = true;
    boolean unclogging = false;

    public static double minHoodAngle = 30;
    public static double maxHoodAngle = 50;
    public static double currHoodAngle = minHoodAngle;

    public static PIDCoefficients closeZonePIDCoefficients = new PIDCoefficients(0.007,0,0);

    public static PIDCoefficients farPIDCoefficients = new PIDCoefficients(0.04,0,0.00007);

    public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.00045,0,0.0004);

    //ALWAYS DECLARE BEFORE INSTANCE!

    boolean gateOpened  = false;

    public static final Shooter INSTANCE = new Shooter();
    public static double lowFlywheelVelo = 700;
    public static double highFlywheelVelo = 850;
    public static double currSpeed = 500;

    public static double hoodOffset;
    public static double shooterOffset;

    public boolean flywheelOn = false;

    private Shooter() { }

    public final MotorEx Shooter1 = new MotorEx("Shooter1");
    private final MotorEx Shooter2 = new MotorEx("Shooter2").reversed();

    private final MotorGroup shooterMotors = new MotorGroup(Shooter1,Shooter2);

    private final ServoEx Gate = new ServoEx("Gate");

    private final ServoEx Hood = new ServoEx("Hood");

    private final ServoEx Kicker = new ServoEx("Kicker");

    public static double closePos = 0.1;
    public static double openPos = 0.3;

    ControlSystem closeFlywheelController = ControlSystem.builder()
            .velPid(closeZonePIDCoefficients)
            .basicFF(flywheelFFCoefficients)
            .build();

    ControlSystem farFlywheelController = ControlSystem.builder()
            .velPid(farPIDCoefficients)
            .basicFF(flywheelFFCoefficients)
            .build();

    public static InterpLUT velolut = new InterpLUT();
    public static   InterpLUT hoodlut = new InterpLUT();


    public final Command On = new InstantCommand(() -> {
        flywheelOn = true;
        unclogging = false;
    });
    public final Command Off = new InstantCommand(() -> flywheelOn = false);


    //Servo Commands

    public final Command openGate = new InstantCommand(() ->{
        Gate.setPosition(openPos);
    }
    );
    public final Command closeGate = new InstantCommand(() ->{
        Gate.setPosition(closePos);
    }
    );

    public final Command Kick = new SequentialGroup(
            new SetPosition(Kicker, kickerOn).thenWait(kickerWait),
            new SetPosition(Kicker, kickerOff)
    );


    //Admin commands

    public Command enableVelo = new InstantCommand(() -> {
        veloLock = false;
    });

    public Command disableVelo = new InstantCommand(() -> {
        veloLock = true;
        shooterOffset = 0;
        hoodOffset = 0;
        //RESETS!
    });

    public Command Init = new ParallelGroup(
            enableVelo,
            closeGate,
            Off
    );


    //Tuning Commands

    public Command setSpeedHigh = new InstantCommand(() -> currSpeed += 50);
    public Command setSpeedLow = new InstantCommand(() -> currSpeed -= 50);


    public Command increaseAngle = new InstantCommand(() -> setHood = Math.min(maxHoodPose, setHood+0.1));
    public Command decreaseAngle = new InstantCommand(() -> setHood = Math.max(0, setHood-0.1));


    //Diagnostic commands

    public Command increaseSpeedOffset = new InstantCommand(() -> shooterOffset += 50);
    public Command decreaseSpeedOffset = new InstantCommand(() -> shooterOffset -= 50);

    public Command increaseAngleOffset = new InstantCommand(() -> hoodOffset = hoodOffset+0.1);
    public Command decreaseAngleOffset = new InstantCommand(() -> hoodOffset = hoodOffset-0.1);


    public final Command UnclogOn = new InstantCommand(() -> unclogging = true);
    public final Command UnclogOff = new InstantCommand(() -> unclogging = false);

    //Fin

    @Override
    public void initialize(){
        Shooter.INSTANCE.Off.schedule();
        openGate.thenWait(0.1).then(closeGate).schedule();

        //Adding each val with a key
        hoodlut = new InterpLUT()
        {{
            add(39, 0.04);
            add(48, 0.34);
            add(52.9, 0.34);
            add(70.9, 0.4);
            add(85, 0.3);
            add(91, 0.29);
            add(121, 0.45);
            add(134, 0.85);

        }};

        velolut = new InterpLUT()
        {{
            add(36.4, 980);
            add(39, 1030);
            add(48, 1080);
            add(52.9, 1080);
            add(66.6, 1160);
            add(67.6, 1160 );
            add(70.9, 1200 );
            add(85, 1160 );
            add(91, 1160 );
            add(121, 1330);
            add(132, 1400);

        }};
//generating final equation
        velolut.createLUT();
        hoodlut.createLUT();
    }

    @Override
    public void periodic() {
        //Kicker.setPosition(KickerOn);


        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(mainConstants.goalPose);

        if(!veloLock){
            currSpeed = velolut.get(distance);
            setHood  = hoodlut.get(distance);
        }

        closeFlywheelController.setGoal(new KineticState(0,currSpeed + shooterOffset,0));
        Hood.setPosition(Math.max(Math.min(maxHoodPose, setHood + hoodOffset),0)); // CLAMPING


        double power = 0;
        if (flywheelOn) {
            power = closeFlywheelController.calculate(shooterMotors.getState());
            shooterMotors.setPower(power);
        }
        else if(unclogging) {
                shooterMotors.setPower(-0.7);
        }
        else{
            shooterMotors.setPower(0);
        }

        //ActiveOpMode.telemetry().addData("power", power);
        ActiveOpMode.telemetry().addData("current velo", shooterMotors.getVelocity());
        ActiveOpMode.telemetry().addData("speed set!", currSpeed);
        ActiveOpMode.telemetry().addData("hood angle", setHood);
        ActiveOpMode.telemetry().addData("distance!", distance);
        ActiveOpMode.telemetry().addData("shooter lock?", veloLock);
        ActiveOpMode.telemetry().addData("hoodOffset", hoodOffset);
        ActiveOpMode.telemetry().addData("shooterOffset", shooterOffset);
    }
}