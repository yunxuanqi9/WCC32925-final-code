package org.firstinspires.ftc.teamcode.dodgyLastMinute;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret(){}

    public static double MAX_TICK_VALUE = 300;
    public static double MIN_TICK_VALUE = -300;

    double pulleyRatio = 8;

    double PPR = 145.1;

    ControlSystem controller = ControlSystem.builder()
            .posPid(0.003,0,0)
            .build();

    private final MotorEx turretMotor = new MotorEx("Turret").zeroed().reversed().brakeMode();

    public static boolean turretLock = false;

    public double getTurretPosition(){
        return turretMotor.getCurrentPosition();
    }

    public Command enableTracking = new InstantCommand(() -> turretLock = false);
    public Command disableTracking = new InstantCommand(() -> turretLock = true);

    public Command setTurretPosition(double pos){
        return new InstantCommand(() -> turretMotor.setCurrentPosition(pos));
    }

    public static Pose goalPose = new Pose(0,144);

    public double calculateTurretPosition(){
        Pose robotPose = PedroComponent.follower().getPose();
        double turretDegrees = (Math.toDegrees(Math.atan2(
                goalPose.getY()- robotPose.getY(),
                goalPose.getX()-robotPose.getPose().getX())
        ) - Math.toDegrees(robotPose.getPose().getHeading()));
        double goalLocation = (turretDegrees/360.0) * PPR * pulleyRatio;
        return Math.max(MIN_TICK_VALUE,Math.min(MAX_TICK_VALUE,goalLocation));
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
    }


}
