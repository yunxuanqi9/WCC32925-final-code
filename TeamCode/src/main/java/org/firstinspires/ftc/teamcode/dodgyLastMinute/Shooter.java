package org.firstinspires.ftc.teamcode.dodgyLastMinute;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.InterpLUT;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
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
    boolean BLUE_TEAM;

    public static double fixedSpeed;

    public static Pose goalPose = new Pose(0,144);

    double PPR = 145.1; // 1150 motor
    double TURRET_LIMIT = 180;

    boolean veloLock = true;

    public static double minHoodAngle = 35;
    public static double maxHoodAngle = 50;
    public static double currHoodAngle = minHoodAngle;
    boolean gateOpened  = false;

    public static final Shooter INSTANCE = new Shooter();
    public static double lowFlywheelVelo = 700;
    public static double highFlywheelVelo = 850;
    public double currSpeed;

    public boolean flywheelOn = false;

    private Shooter() { }

    public final MotorEx Shooter1 = new MotorEx("Shooter1");
    private final MotorEx Shooter2 = new MotorEx("Shooter2").reversed();

    private final MotorGroup shooterMotors = new MotorGroup(Shooter1,Shooter2);

    private final ServoEx Gate = new ServoEx("Gate");


    public static double closePos = 0.4;
    public static double openPos = 0.8;


    public static double p = 0.05; // CONFIGURE

    public static double kV = 1;
    public static double kS = 1;




    ControlSystem FlywheelController = ControlSystem.builder()
            .velPid(0.05, 0, 0)
            .basicFF(0,0,0.00045)
            .build();

    InterpLUT lut = new InterpLUT();

    public final Command On = new InstantCommand(() -> flywheelOn = true);
    public final Command Off = new InstantCommand(() -> flywheelOn = false);


    public final Command openGate = new InstantCommand( () ->{
            Gate.setPosition(openPos);
    }
    );
    public final Command closeGate = new InstantCommand( () ->
            Gate.setPosition(closePos)
    );

    public Command toggleVeloLock = new InstantCommand(() -> veloLock = !veloLock);

    public Command setSpeedLow = new InstantCommand(() -> currSpeed = lowFlywheelVelo);
    public Command setSpeedHigh = new InstantCommand(() -> currSpeed = highFlywheelVelo);

    private ElapsedTime timer;


    @Override
    public void initialize(){

        if(!BLUE_TEAM){
            goalPose.mirror();
        }

        timer = new ElapsedTime();
        //Adding each val with a key
        lut = new InterpLUT()
        {{
            add(24, 750);
            add(60, 840);
            add(84, 910 );
            add(120, 1075);
            add(144, 1215);
        }};
//generating final equation
        lut.createLUT();

    }

    @Override
    public void periodic() {

        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(goalPose);

        if(!veloLock){
            currSpeed = lut.get(distance);
            }

        FlywheelController.setGoal(new KineticState(0,currSpeed,0));

        double power = 0;
        if (flywheelOn) {
            power = FlywheelController.calculate(shooterMotors.getState());
            shooterMotors.setPower(power);

        } else {
            shooterMotors.setPower(0);
        }

        ActiveOpMode.telemetry().addData("power", power);
        ActiveOpMode.telemetry().addData("current velo", shooterMotors.getVelocity());
        ActiveOpMode.telemetry().addData("speed set!", currSpeed);
        ActiveOpMode.telemetry().addData("shooter lock?", veloLock);

    }
}