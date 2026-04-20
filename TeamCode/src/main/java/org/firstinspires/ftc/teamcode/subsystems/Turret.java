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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {


    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.018,0,0.0005);
    public static BasicFeedforwardParameters turretFFCoefficients = new BasicFeedforwardParameters(0.001,0,0.000001);


    public static final Turret INSTANCE = new Turret();


    ControlSystem controller = ControlSystem.builder()
            .posPid(turretCoefficients) //DO NOT. UNDER ANY CIRCUMSTANCES USE VARIABLES!!!!
            .basicFF(turretFFCoefficients)
            .build();


    private Turret(){}

    public static double MAX_TICK_VALUE = 310;
    public static double MIN_TICK_VALUE = -310;

    public static boolean SOTM = false;

    private InterpLUT flighttimelut;

    //public static double MAX_TICK_VALUE = 380;
    //public static double MIN_TICK_VALUE = -310;

    double pulleyRatio = 8;

    double PPR = 145.1;


    private final MotorEx turretMotor = new MotorEx("Turret").zeroed().brakeMode().reversed()
            ;
    public static boolean turretLock = true;

    public double getTurretPosition(){
        return turretMotor.getCurrentPosition();
    }

    public Command enableTracking = new InstantCommand(() -> {
        turretLock = false;
    });
    public Command disableTracking = new InstantCommand(() -> turretLock = true);

    public Command setTurretPosition(double pos){
        return new InstantCommand(() -> turretMotor.setCurrentPosition(pos));
    }

    public double turretDegrees;
    public double goalLocation;

    public double calculateTurretPosition(Pose goalPose){
        Pose robotPose = PedroComponent.follower().getPose();
        double rawDelta = (Math.atan2(
                goalPose.getY()- robotPose.getY(),
                goalPose.getX()-robotPose.getX())
        ) - robotPose.getPose().getHeading();
        turretDegrees = Math.toDegrees(rawDelta);
        double normalisedDelta = Math.atan2(Math.sin(rawDelta), Math.cos(rawDelta));
        goalLocation = (Math.toDegrees(normalisedDelta)/360) * PPR * pulleyRatio;
        return Math.max(MIN_TICK_VALUE,Math.min(MAX_TICK_VALUE,goalLocation)); //CLAMPING
    }




    public double flightTime(){
        return 1;
    }

    public double calcSOTMTurretPosition(Pose goalPose){
        Pose robotPose = PedroComponent.follower().getPose();

        double flightTime = flighttimelut.get(robotPose.distanceFrom(goalPose));

        Pose virtualGoalPose = new Pose(goalPose.getX() + PedroComponent.follower().getVelocity().getXComponent()*flightTime,
                goalPose.getY() + PedroComponent.follower().getVelocity().getYComponent()*flightTime,
                robotPose.getHeading()
        );

        turretDegrees = (Math.toDegrees(Math.atan2(
                goalPose.getY()- robotPose.getY(),
                goalPose.getX()-robotPose.getPose().getX())
        ) - Math.toDegrees(robotPose.getPose().getHeading()));


        double goalLocation = (turretDegrees/360.0) * PPR * pulleyRatio;
        return Math.max(MIN_TICK_VALUE,Math.min(MAX_TICK_VALUE,goalLocation)); //CLAMPING
    }


    public void initialize(){
        turretLock = true;

        if(mainConstants.autoEndX == 0 && mainConstants.autoEndY == 0 && mainConstants.autoEndHeading == 0){
            turretMotor.zero();
        }

        //backup
        flighttimelut = new InterpLUT()
        {{
            //REPLACE WITH ACTUAL DATA
            add(24, 0.6);
            add(60.6, 0.8);
            add(84, 1);
            add(120, 1.5);
            add(144, 2);
        }};
    }

    public void periodic(){
        double targetPos;

        if(SOTM){
            targetPos = calcSOTMTurretPosition(mainConstants.goalPose);
        }
        else{
            targetPos = calculateTurretPosition(mainConstants.goalPose);
        }

        //SOTM?

        controller.setGoal(new KineticState(targetPos,0,0));
        double power;
        if(turretLock){
            power = 0;
        }
        else{
            power = controller.calculate(turretMotor.getState());
        }

        turretMotor.setPower(power);
        ActiveOpMode.telemetry().addData("current motor value",turretMotor.getCurrentPosition());
        ActiveOpMode.telemetry().addData("current VELO",turretMotor.getVelocity());
        ActiveOpMode.telemetry().addData("Turret locked?",turretLock);
        ActiveOpMode.telemetry().addData("turret degrees",turretDegrees);
        ActiveOpMode.telemetry().addData("goal location",goalLocation);

        ActiveOpMode.telemetry().addData("Goal pose X",mainConstants.goalPose.getX());
        ActiveOpMode.telemetry().addData("Goal pose Y",mainConstants.goalPose.getY());

    }


}
