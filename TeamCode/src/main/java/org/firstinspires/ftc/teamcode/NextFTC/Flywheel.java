package org.firstinspires.ftc.teamcode.NextFTC;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * The flywheel subsystem implemented from
 * <a href="https://v1.nextftc.dev/guide/subsystems/flywheel">docs</a>.
 */
@Configurable
public class    Flywheel implements Subsystem {

    boolean BLUE_TEAM;
    double PPR = 145.1; // 1150 motor
    double TURRET_LIMIT = 180;

    double minHoodAngle = 35;
    double maxHoodAngle = 50;
    double currHoodAngle = minHoodAngle;

    boolean gateOpened  = false;


    public static final Flywheel INSTANCE = new Flywheel();
    public static double FlywheelVelo = 1000;
    public static double flywheelPower = 0.6;

    public boolean flywheelOn;

    private Flywheel() { }


    public final MotorEx Shooter1 = new MotorEx("Shooter1");
    public final MotorEx Shooter2 = new MotorEx("Shooter2").reversed();
    private final MotorEx Turret = new MotorEx("Turret").zeroed().brakeMode();
    private final ServoEx Gate = new ServoEx("Gate");
    private final ServoEx Hood = new ServoEx("Hood");


    public static double closePos = 0.25;
    public static double openPos = 0.8;


    public static double p = 0.05; // CONFIGURE
    public static double turretP = 0.05;

    private final ControlSystem TurretController = ControlSystem.builder()
            .posPid(turretP,0,0)
            .build();

    private final ControlSystem FlywheelController = ControlSystem.builder()
            .velPid(p, 0, 0)
            .build();

    PController controller = new PController(0.05);


    InterpLUT lut = new InterpLUT();

    public void On(){
        flywheelOn = true;
        //FlywheelController.setGoal(new KineticState(0,FlywheelVelo));
        Shooter1.setPower(flywheelPower);
        Shooter2.setPower(flywheelPower);
    }

    public void Off(){
        flywheelOn = false;
        //FlywheelController.setGoal(new KineticState(0,0));
        Shooter1.setPower(0);
        Shooter2.setPower(0);
    }

    /* public final Command off = new RunToVelocity(controller, 0.0);
    public final Command on = new RunToVelocity(controller, FlywheelVelo);*/


    public void openGate(){
        Gate.setPosition(openPos);
        gateOpened = true;
    }

    public void closeGate(){
        Gate.setPosition(closePos);
        gateOpened = false;
    }

    //Weird turret shit.

    public Pose calcGoalPose(boolean SOTM, Follower drive){
        Pose defaultgoalPose = new Pose(16,140,0);
        if(!BLUE_TEAM){
            //mirrors if red team.
            defaultgoalPose.mirror();
        }

        double goalx = defaultgoalPose.getX();
        double goaly = defaultgoalPose.getY();

        if(SOTM){
            goalx -= drive.getVelocity().getXComponent()*flightTime(drive,defaultgoalPose);
            goaly -= drive.getVelocity().getYComponent()*flightTime(drive,defaultgoalPose);
        }

        //returns virtual goal pose
        return new Pose(goalx,goaly,0);
    }

    public double flightTime(Follower drive, Pose goalPose){
        //MUST CONVERT TO INCHES
        double ballSpeed = ((FlywheelVelo/60)*96)/25.4; //INCHES PER SECOND
        double horizontal = ballSpeed*Math.cos(Math.toRadians(currHoodAngle));
        return drive.getPose().distanceFrom(goalPose)/horizontal;
    }

    private double distFromGoal(Follower drive){
        Pose goalPose = calcGoalPose(false,drive);
        // gets distance between goal.
        return Math.hypot(goalPose.getX()-drive.getPose().getX(),
                goalPose.getY()-drive.getPose().getY());
    }

    public double rotationalAimbotLocalizer (Follower drivetrain){
        Pose goalPose = calcGoalPose(false, drivetrain);

        return (Math.toDegrees(Math.atan2(
                goalPose.getY()- drivetrain.getPose().getY(),
                goalPose.getX()-drivetrain.getPose().getX())
        ) - Math.toDegrees(drivetrain.getPose().getHeading()) + 90)%360;
    }

    public void setTurretAngle(double angle){
        double safetyAngle = Math.min(Math.max(0,angle),TURRET_LIMIT);
        TurretController.setGoal(new KineticState((safetyAngle/360)*PPR));
    }

    public void setPosDebug(double pos){
        TurretController.setGoal(new KineticState(pos));
    }

    public void updateAimbot(Follower drive){
        setTurretAngle(Math.min(Math.max(0,rotationalAimbotLocalizer(drive)),TURRET_LIMIT));
    }

    public double getPos(){
        return Turret.getCurrentPosition();
    }


    double hoodRatio = 11.875;

    public double hoodPos(){
        return Hood.getPosition();
    }

    public void setHoodAngle(double angle){
        currHoodAngle = minHoodAngle + Math.min(Math.max((angle-minHoodAngle)/300, 0),0.5);
        Hood.setPosition(Math.min(Math.max((angle-minHoodAngle/hoodRatio)/300, 0),0.5));
    }

    private ElapsedTime timer;


    public void setFlywheelPower(double power){
        flywheelPower = power;
    }

    public void autoFlywheelPower(Follower follower){
        Pose goalPose = new Pose(0,144,0);
        if(!BLUE_TEAM){
            goalPose.mirror();
        }
        double distance = follower.getPose().distanceFrom(goalPose);
        flywheelPower = (lut.get(distance));
    }

    @Override
    public void initialize(){
        Turret.setPower(0);
        Turret.setCurrentPosition(25);
        timer = new ElapsedTime();
        //Adding each val with a key
        lut = new InterpLUT()
        {{
            add(10, 0.25);
            add(30, 0.30 );
            add(67, 0.35);
            add(100, 0.5);
            add(120, 0.6);
        }};
//generating final equation
        lut.createLUT();

    }

    @Override
    public void periodic() {
        //Shooter1.setPower(FlywheelController.calculate(Shooter1.getState()));
        //Shooter2.setPower(FlywheelController.calculate(Shooter1.getState()));
        //Shooter1.setPower(FlywheelController.calculate(new KineticState(0,-Shooter1.getVelocity())));
        //Shooter2.setPower(FlywheelController.calculate(new KineticState(0,-Shooter1.getVelocity())));
        //Turret.setPower(TurretController.calculate(Turret.getState()));

    }
}