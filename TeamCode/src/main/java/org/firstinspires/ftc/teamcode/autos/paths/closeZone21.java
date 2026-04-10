package org.firstinspires.ftc.teamcode.autos.paths;


import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitGate;
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
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing Blue 18 Ball Optimised", group = "Autonomous")
@Configurable // Panels
public class closeZone21 extends NextFTCOpMode {
    public closeZone21() {
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

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
        follower().setStartingPose(new Pose(18.129825124338204, 120.01719878068347, Math.toRadians(144)));
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

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(middleSpike),
                Intake.INSTANCE.Off,
                new FollowPath(middleScore),
                shootArtifacts(),


                Intake.INSTANCE.On,
                new FollowPath(gateIntake1),
                Intake.INSTANCE.Off,

                new FollowPath(gateScore1),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(gateIntake2),
                Intake.INSTANCE.Off,

                new FollowPath(gateScore2),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(gateIntake3),
                Intake.INSTANCE.Off,

                new FollowPath(gateScore3),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(topSpike),
                Intake.INSTANCE.Off,

                new FollowPath(spikeScore),
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

    public PathChain shootPreload;
    public PathChain middleSpike;
    public PathChain middleScore;
    public PathChain gateIntake1;
    public PathChain gateScore1;
    public PathChain gateIntake2;
    public PathChain gateScore2;
    public PathChain gateIntake3;
    public PathChain gateScore3;
    public PathChain topSpike;
    public PathChain spikeScore;


    public void buildPaths() {
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.130, 120.017),

                                new Pose(50.087, 95.583)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(134))

                .build();

        middleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.087, 95.583),
                                new Pose(54.609, 81.861),
                                new Pose(58.835, 57.278),
                                new Pose(24.870, 60.365)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .build();

        middleScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.870, 60.365),

                                new Pose(57.363, 79.810)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        gateIntake1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.363, 79.810),

                                new Pose(11.440, 62.147)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.440, 62.147),

                                new Pose(57.136, 80.269)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        gateIntake2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.136, 80.269),

                                new Pose(11.382, 61.865)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.382, 61.865),

                                new Pose(56.781, 79.171)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        gateIntake3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.781, 79.171),

                                new Pose(11.858, 62.591)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.858, 62.591),

                                new Pose(57.414, 80.315)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(57.414, 80.315),

                                new Pose(24.087, 83.443)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                .build();

        spikeScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.087, 83.443),

                                new Pose(40.030, 99.542)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();
    }

}