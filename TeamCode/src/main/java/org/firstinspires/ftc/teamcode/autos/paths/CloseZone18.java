package org.firstinspires.ftc.teamcode.autos.paths;


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


@Autonomous(name = "Pedro Pathing Blue 18 Ball", group = "Autonomous")
@Configurable // Panels
public class CloseZone18 extends NextFTCOpMode {

    public CloseZone18() {
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
        Shooter.INSTANCE.closeGate.schedule();
        Shooter.INSTANCE.Off.schedule();
        Turret.INSTANCE.enableTracking.afterTime(0.01).schedule();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        buildPaths();
        follower().setStartingPose(new Pose(23.44347826086957, 126.74782608695654, Math.toRadians(144)));
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);


    }
    @Override
    public void onStartButtonPressed(){
        autonomousRoutine().run();
    }


    public Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On,

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(intakeMiddleSpike),
                Intake.INSTANCE.Off,
                new FollowPath(shootMiddleSpiike),
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
                new FollowPath(intakeTopSpike),
                Intake.INSTANCE.Off,

                new FollowPath(shootTopSpike),
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
    public PathChain intakeMiddleSpike;
    public PathChain shootMiddleSpiike;
    public PathChain gateIntake1;
    public PathChain gateScore1;
    public PathChain gateIntake2;
    public PathChain gateScore2;
    public PathChain gateIntake3;
    public PathChain gateScore3;
    public PathChain intakeTopSpike;
    public PathChain shootTopSpike;

    public void buildPaths() {
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.443, 126.748),

                                new Pose(50.087, 95.583)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(134))

                .build();

        intakeMiddleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.087, 95.583),
                                new Pose(54.609, 81.861),
                                new Pose(58.835, 57.278),
                                new Pose(24.870, 60.365)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .setReversed()
                .build();

        shootMiddleSpiike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.870, 60.365),

                                new Pose(58.426, 75.913)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();

        gateIntake1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.426, 75.913),

                                new Pose(13.565, 60.730)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.565, 60.730),

                                new Pose(55.896, 77.435)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        gateIntake2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.896, 77.435),

                                new Pose(13.330, 61.157)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.330, 61.157),

                                new Pose(55.896, 77.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        gateIntake3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.896, 77.400),

                                new Pose(13.452, 61.174)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(160))

                .build();

        gateScore3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.452, 61.174),

                                new Pose(56.174, 77.304)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(130))

                .build();

        intakeTopSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.174, 77.304),

                                new Pose(24.087, 83.443)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))

                .build();

        shootTopSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.087, 83.443),

                                new Pose(37.452, 96.661)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                .build();
    }

}