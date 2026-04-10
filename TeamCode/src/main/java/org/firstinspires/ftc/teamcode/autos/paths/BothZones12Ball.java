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


@Autonomous(name = "Pedro Pathing Blue 12 Ball", group = "Autonomous")
@Configurable // Panels
public class BothZones12Ball extends NextFTCOpMode {
    public BothZones12Ball() {
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
        follower().setStartingPose(new Pose(23.089234718434138, 120.72568586555433, Math.toRadians(144)));
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

                new FollowPath(ShootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(TopSpikeIntake),
                Intake.INSTANCE.Off,
                new FollowPath(Score1),
                shootArtifacts(),


                Intake.INSTANCE.On,
                new FollowPath(MidSpikeIntake),
                Intake.INSTANCE.Off,
                new FollowPath(Score2),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(BottomSpikeIntake),
                Intake.INSTANCE.Off,
                new FollowPath(Score3),
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

    public PathChain ShootPreload;
    public PathChain TopSpikeIntake;
    public PathChain Score1;
    public PathChain MidSpikeIntake;
    public PathChain Score2;
    public PathChain BottomSpikeIntake;
    public PathChain Score3;

    public void buildPaths() {
        ShootPreload = follower().pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.824, 119.279),

                                    new Pose(50.450, 88.705)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(144))

                    .build();

        TopSpikeIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.450, 88.705),

                                new Pose(15.930, 83.791)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(179), Math.toRadians(180))

                .build();

        Score1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.930, 83.791),

                                new Pose(53.765, 82.096)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        MidSpikeIntake = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(53.765, 82.096),
                                new Pose(47.691, 82.022),
                                new Pose(53.413, 68.030),
                                new Pose(50.076, 56.848),
                                new Pose(9.928, 59.755)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Score2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.928, 59.755),
                                new Pose(50.757, 35.861),
                                new Pose(62.400, 22.330)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        BottomSpikeIntake = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.400, 22.330),
                                new Pose(42.791, 38.352),
                                new Pose(15.513, 35.417)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Score3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.513, 35.417),

                                new Pose(51.261, 10.730)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

}