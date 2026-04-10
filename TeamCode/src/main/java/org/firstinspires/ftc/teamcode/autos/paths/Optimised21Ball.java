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


@Autonomous(name = "Pedro Pathing Blue 21 Ball Optimised", group = "Autonomous")
@Configurable // Panels
public class Optimised21Ball extends NextFTCOpMode {
    public Optimised21Ball() {
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

    private Pose startingPose = new Pose(39.662955238248024, 104.75597625541474, Math.toRadians(144));

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

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(topSpike),
                Intake.INSTANCE.Off,
                new FollowPath(spikeScore1),
                shootArtifacts(),


                Intake.INSTANCE.On,
                new FollowPath(midSpike),
                Intake.INSTANCE.Off,
                new FollowPath(spikeScore2),
                shootArtifacts(),

                //rinse and repeat.
                Intake.INSTANCE.On,
                new FollowPath(openGate),
                new FollowPath(waitToIntake),
                Intake.INSTANCE.Off,
                new FollowPath(gateScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(openGate),
                new FollowPath(waitToIntake),
                Intake.INSTANCE.Off,
                new FollowPath(gateScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(openGate),
                new FollowPath(waitToIntake),
                Intake.INSTANCE.Off,
                new FollowPath(gateScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(openGate),
                new FollowPath(waitToIntake),
                Intake.INSTANCE.Off,
                new FollowPath(gateScore),
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
    public PathChain topSpike;
    public PathChain spikeScore1;
    public PathChain midSpike;
    public PathChain spikeScore2;
    public PathChain openGate;
    public PathChain waitToIntake;
    public PathChain gateScore;

    public void buildBluePaths() {
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.580, 119.621),

                                new Pose(37.715, 104.756)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(37.715, 104.756),
                                new Pose(43.641, 82.315),
                                new Pose(24.661, 84.365)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .build();

        spikeScore1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.661, 84.365),

                                new Pose(53.491, 92.280)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))

                .build();

        midSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(53.491, 92.280),
                                new Pose(29.865, 60.930),
                                new Pose(19.129, 59.336)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        spikeScore2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.129, 59.336),

                                new Pose(53.314, 91.572)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(53.314, 91.572),

                                new Pose(12.236, 62.523)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(160))

                .build();

        waitToIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.236, 62.523),

                                new Pose(12.675, 54.399)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(145))
                .setReversed()
                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.675, 54.399),

                                new Pose(53.314, 91.572)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    public void buildRedPaths(){
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.420, 119.621),

                                new Pose(106.285, 104.756)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(106.285, 104.756),
                                new Pose(84.772, 79.481),
                                new Pose(119.339, 84.365)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))

                .build();

        spikeScore1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(119.339, 84.365),

                                new Pose(95.646, 93.697)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))

                .build();

        midSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(95.646, 93.697),
                                new Pose(114.135, 60.930),
                                new Pose(124.871, 59.336)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        spikeScore2 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.871, 59.336),

                                new Pose(95.646, 93.166)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(95.646, 93.166),

                                new Pose(131.764, 62.523)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(20))

                .build();

        waitToIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.764, 62.523),

                                new Pose(131.325, 54.399)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(35))
                .setReversed()
                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.325, 54.399),

                                new Pose(95.469, 93.343)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}