package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.TeleOpTest.waitGate;
import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.TeleOpTest.waitShoot;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BlueAUTOjade extends NextFTCOpMode {

    BlueAUTOjade() {
        addComponents(
                new SubsystemComponent(
                        Turret.INSTANCE,
                        Shooter.INSTANCE,
                        Intake.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
    }


    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime autoTimer = new ElapsedTime();


    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
        follower().setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = 0;
    }

    public Command shootArtifacts() {
        return new SequentialGroup(
                new SequentialGroup(
                        Shooter.INSTANCE.openGate,
                        new Delay(waitGate),
                        Intake.INSTANCE.On,
                        new Delay(waitShoot)
                ),
                new ParallelGroup(
                        Shooter.INSTANCE.closeGate,
                        Intake.INSTANCE.Off
                )
        );
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(Path1),
                Shooter.INSTANCE.On,
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(Path2),

                Intake.INSTANCE.Off,
                new FollowPath(Path3),

                shootArtifacts(),
                new FollowPath(Path4),

                Intake.INSTANCE.On
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

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;

    public void buildPaths() {
        Path1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.895, 130.612),


                                new Pose(58.332, 96.901)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(133))
                .build();
        Path2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.332, 96.901),
                                new Pose(51.568, 80.692),
                                new Pose(17.754, 83.182)
                        )
                ).setTangentHeadingInterpolation()
                .build();
        Path3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.754, 83.182),


                                new Pose(66.055, 80.252)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(128))
                .setReversed()
                .build();
        Path4 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(66.055, 80.252),
                                new Pose(47.691, 82.022),
                                new Pose(53.413, 68.030),
                                new Pose(30.770, 52.065),
                                new Pose(13.963, 62.833)
                        )
                ).setTangentHeadingInterpolation()


                .build();


        Path5 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.963, 62.833),
                                new Pose(50.757, 35.861),
                                new Pose(62.400, 22.330)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(120))
                .setReversed()
                .build();


        Path6 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.400, 22.330),
                                new Pose(42.791, 38.352),
                                new Pose(15.513, 35.417)
                        )
                ).setTangentHeadingInterpolation()
                .build();
        Path7 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.513, 35.417),


                                new Pose(51.261, 10.730)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(110))
                .setReversed()
                .build();
    }

}