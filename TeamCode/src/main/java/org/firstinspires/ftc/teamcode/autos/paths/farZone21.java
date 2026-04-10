package org.firstinspires.ftc.teamcode.autos.paths;

import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitGate;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotConstants.mainConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

public class farZone21 extends NextFTCOpMode {
    public farZone21() {
        addComponents(
                new SubsystemComponent(
                        Turret.INSTANCE,
                        Intake.INSTANCE,
                        Shooter.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private Pose startingPose = new Pose(56.42047168297771, 8.86635648965186, Math.toRadians(144));

    @Override
    public void onInit() {
        Shooter.INSTANCE.initShooter.schedule();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        if(mainConstants.redTeam){
            buildRedPaths();
            startingPose = startingPose.mirror(); // MIRRORS POSE!!!!
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


    public Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On,

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


                //Intake from Gate and score.
                Intake.INSTANCE.On,
                new FollowPath(openGate),
                new FollowPath(gateIntake),
                Intake.INSTANCE.Off,
                new FollowPath(scoreGate),
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


    @Override
    public void onUpdate() {
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("X", follower().getPose().getX());
        panelsTelemetry.debug("Y", follower().getPose().getY());
        panelsTelemetry.debug("Heading", follower().getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop(){
        mainConstants.autoEndPose = follower().getPose();
        ActiveOpMode.telemetry().addData("End pose X",mainConstants.autoEndPose.getX());
        ActiveOpMode.telemetry().addData("End pose Y",mainConstants.autoEndPose.getY());
    }

    public Command shootArtifacts(){
        return new ParallelGroup(
                new SequentialGroup(
                        Shooter.INSTANCE.openGate.thenWait(waitGate),
                        Shooter.INSTANCE.closeGate
                ),
                new SequentialGroup(
                        Intake.INSTANCE.Nudge.thenWait(0.1),
                        Intake.INSTANCE.On.thenWait(1),
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
                                new Pose(56.420, 8.866),

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

    public void buildRedPaths(){
        //ADD MIRRORED PATHS!
    }
}
