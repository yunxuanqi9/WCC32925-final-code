package org.firstinspires.ftc.teamcode.autos.paths;


import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitGate;
import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitToKick;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;


import org.firstinspires.ftc.teamcode.robotConstants.mainConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing 21 Ball Corner + Gate", group = "Autonomous")
@Configurable // Panels
public abstract class CornerIntake21 extends NextFTCOpMode {

    protected final boolean redTeam;

    public CornerIntake21(Boolean redTeam) {
        addComponents(
                new SubsystemComponent(
                        Turret.INSTANCE,
                        Intake.INSTANCE,
                        Shooter.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
        this.redTeam = redTeam;
        mainConstants.redTeam = redTeam;
    }

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private Pose startingPose = new Pose(64.39095138777475, 8.157869404781021, Math.toRadians(180));
    private Pose openGatePose = mainConstants.gateIntake;

    @Override
    public void onInit() {
        mainConstants.setAlliance(redTeam);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        if(redTeam){
            buildRedPaths();
            startingPose = startingPose.mirror(); // MIRRORS POSE!!!!
            openGatePose = openGatePose.mirror();
        }
        else{
            buildBluePaths();
        }

        follower().setStartingPose(startingPose);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }
    
    @Override
    public void onStartButtonPressed(){
        Shooter.INSTANCE.closeGate.schedule();
        Shooter.INSTANCE.Off.schedule();
        Turret.INSTANCE.enableTracking.afterTime(0.01).schedule();

        autonomousRoutine().run();
    }
    
    @Override
    public void onUpdate() {
        // Log values to Panels and Driver Station

        Pose robotPose = follower().getPose();

        //Constantly edits

        if(robotPose.getX() != 0 && robotPose.getY() != 0 && robotPose.getHeading()!=0){
            mainConstants.autoEndX = robotPose.getX();
            mainConstants.autoEndY = robotPose.getY();
            mainConstants.autoEndHeading = robotPose.getHeading();
        }

        panelsTelemetry.debug("X", follower().getPose().getX());
        panelsTelemetry.debug("Y", follower().getPose().getY());
        panelsTelemetry.debug("Heading", follower().getPose().getHeading());

        panelsTelemetry.debug("MainconstantsRed?", mainConstants.redTeam);
        panelsTelemetry.debug("this.Red?", this.redTeam);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop(){
        mainConstants.autoEndPose = follower().getPose();
        ActiveOpMode.telemetry().addData("End pose X",mainConstants.autoEndPose.getX());
        ActiveOpMode.telemetry().addData("End pose Y",mainConstants.autoEndPose.getY());
    }


    public Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On.thenWait(3),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(startToIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(cornerIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(cornerIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(cornerIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(cornerIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(cornerIntake),
                Intake.INSTANCE.Off,
                new FollowPath(farZoneScore),
                shootArtifacts()
        );
    }

    public Command shootArtifacts(){
        return new ParallelGroup(
                new SequentialGroup(
                        Shooter.INSTANCE.openGate.thenWait(waitGate),
                        Shooter.INSTANCE.closeGate
                ),
                new SequentialGroup(
                        Intake.INSTANCE.On.thenWait(waitToKick),
                        Shooter.INSTANCE.Kick,
                        Intake.INSTANCE.Off
                )
        );
    }


    public PathChain startToIntake;
    public PathChain farZoneScore;
    public PathChain cornerIntake;
    public PathChain openGate;
    public PathChain gateIntake;
    public PathChain scoreGate;

    public void buildBluePaths() {
        startToIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                startingPose,

                                new Pose(8.137, 10.910)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        farZoneScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.137, 10.910),

                                new Pose(60.328, 11.328)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        cornerIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.328, 11.328),

                                new Pose(7.756, 11.494)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.195, 10.786),

                                new Pose(11.461, 60.863)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))

                .build();

        gateIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.461, 60.863),

                                new Pose(12.679, 46.118)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(145))

                .build();

        scoreGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.679, 46.118),

                                new Pose(60.328, 11.328)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void buildRedPaths() {
        //ADD RED PATHS!
    }
}


