package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

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


    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.02,0,0.0008);
    public static BasicFeedforwardParameters turretFFCoefficients = new BasicFeedforwardParameters(0.001,0,0.000001);


    public static final Turret INSTANCE = new Turret();


    ControlSystem controller = ControlSystem.builder()
            .posPid(turretCoefficients) //DO NOT. UNDER ANY CIRCUMSTANCES USE VARIABLES!!!!
            .basicFF(turretFFCoefficients)
            .build();


    private Turret(){}

    public static double MAX_TICK_VALUE = 310;
    public static double MIN_TICK_VALUE = -270;


    //public static double MAX_TICK_VALUE = 380;
    //public static double MIN_TICK_VALUE = -310;

    double pulleyRatio = 8;

    double PPR = 145.1;


    private final MotorEx turretMotor = new MotorEx("Turret").zeroed().brakeMode().reversed()
            ;
    public static boolean turretLock = false;

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

    public static Pose goalPose = new Pose(0,144);


    public double turretDegrees;

    public double calculateTurretPosition(){
        Pose robotPose = PedroComponent.follower().getPose();
        turretDegrees = (Math.toDegrees(Math.atan2(
                goalPose.getY()- robotPose.getY(),
                goalPose.getX()-robotPose.getPose().getX())
        ) - Math.toDegrees(robotPose.getPose().getHeading()));
        double goalLocation = (turretDegrees/360.0) * PPR * pulleyRatio;
        return Math.max(MIN_TICK_VALUE,Math.min(MAX_TICK_VALUE,goalLocation));
    }

    public void init(){
        turretMotor.zero();
    }

    public void periodic(){
        double targetPos = calculateTurretPosition();
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
        ActiveOpMode.telemetry().addData("current power",power);
        ActiveOpMode.telemetry().addData("Turret locked?",turretLock);
        ActiveOpMode.telemetry().addData("turret degrees",turretDegrees);

    }


}
