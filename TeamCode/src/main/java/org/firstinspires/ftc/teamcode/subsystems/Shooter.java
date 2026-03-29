package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
@Configurable
public class Shooter implements Subsystem {
    boolean BLUE_TEAM;

    public static double fixedSpeed;

    public static double setHood = 0; // CHANGE IN PANELS
    public static double maxHoodPose = 0.68;

    public static Pose goalPose = new Pose(0,144);

    double PPR = 145.1; // 1150 motor
    double TURRET_LIMIT = 180;

    boolean veloLock = true;

    public static double minHoodAngle = 30;
    public static double maxHoodAngle = 50;
    public static double currHoodAngle = minHoodAngle;




    public static PIDCoefficients flywheelCoefficients = new PIDCoefficients(0.018,0,0.00007);
    public static BasicFeedforwardParameters flywheelFFCoefficients = new BasicFeedforwardParameters(0.0007,0,0.0001);

    //ALWAYS DECLARE BEFORE INSTANCE!

    boolean gateOpened  = false;

    public static final Shooter INSTANCE = new Shooter();
    public static double lowFlywheelVelo = 700;
    public static double highFlywheelVelo = 850;
    public static double currSpeed = 100;

    public boolean flywheelOn = false;

    private Shooter() { }

    public final MotorEx Shooter1 = new MotorEx("Shooter1");
    private final MotorEx Shooter2 = new MotorEx("Shooter2").reversed();

    private final MotorGroup shooterMotors = new MotorGroup(Shooter1,Shooter2);

    private final ServoEx Gate = new ServoEx("Gate");

    private final ServoEx Hood = new ServoEx("Hood");



    public static double closePos = 0.6;
    public static double openPos = 0.78;


    ControlSystem FlywheelController = ControlSystem.builder()
            .velPid(flywheelCoefficients)
            .basicFF(flywheelFFCoefficients)
            .build();

    InterpLUT velolut = new InterpLUT();
    InterpLUT hoodlut = new InterpLUT();


    public final Command On = new InstantCommand(() -> flywheelOn = true);
    public final Command Off = new InstantCommand(() -> flywheelOn = false);


    public final Command openGate = new InstantCommand(() ->{
        Gate.setPosition(openPos);
    }
    );
    public final Command closeGate = new InstantCommand(() ->{
        Gate.setPosition(closePos);
    }
    );

    public final Command turnHood = new InstantCommand(() ->{
        Gate.setPosition(setHood);
    }
    );

    public Command setSpeedLow = new InstantCommand(() -> currSpeed = lowFlywheelVelo);
    public Command setSpeedHigh = new InstantCommand(() -> currSpeed = highFlywheelVelo);


    @Override
    public void initialize(){

        if(!BLUE_TEAM){
            goalPose.mirror();
        }
        //Adding each val with a key
        hoodlut = new InterpLUT()
        {{
            add(60.6, 0.1);
            add(70, 0.2);

        }};

        velolut = new InterpLUT()
        {{
            add(24, 750);
            add(60.6, 700);
            add(84, 910 );
            add(120, 1075);
            add(144, 1215);
        }};
//generating final equation
        velolut.createLUT();
        hoodlut.createLUT();


        closeGate.schedule();
    }

    @Override
    public void periodic() {

        Hood.setPosition(setHood);

        Pose robotPose = PedroComponent.follower().getPose();

        double distance = robotPose.distanceFrom(goalPose);

        if(!veloLock){
            currSpeed = velolut.get(distance);
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
        ActiveOpMode.telemetry().addData("distance!", distance);
        ActiveOpMode.telemetry().addData("shooter lock?", veloLock);

    }
}