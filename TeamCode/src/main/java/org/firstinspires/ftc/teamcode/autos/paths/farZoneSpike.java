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


@Autonomous(name = "Pedro far Zone spike", group = "Autonomous")
@Configurable // Panels
public abstract class farZoneSpike extends NextFTCOpMode {

    protected final boolean redTeam;

    public farZoneSpike(Boolean redTeam) {
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

    private Pose startingPose = new Pose(56.597593454195426, 0, Math.toRadians(90));
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
        Shooter.INSTANCE.Init.schedule();
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
                Shooter.INSTANCE.On.thenWait(6),

                shootArtifacts(),


                Intake.INSTANCE.On,
                new FollowPath(bottomSpike),
                new FollowPath(Path8),
                Intake.INSTANCE.Off,
                new FollowPath(scoreBottom),
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


    public PathChain bottomSpike;
    public PathChain Path8;
    public PathChain scoreBottom;
    public PathChain startToIntake;
    public PathChain farZoneScore;
    public PathChain cornerIntake;
    public PathChain openGate;
    public PathChain scoreGate;

    public void buildBluePaths() {
        bottomSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.598, 7.598),

                                new Pose(41.173, 35.373)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path8 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(41.173, 35.373),

                                new Pose(19.129, 35.620)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreBottom = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.129, 35.620),

                                new Pose(56.600, 7.800)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        startToIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.600, 7.800),

                                new Pose(9.908, 10.201)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        farZoneScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.908, 10.201),

                                new Pose(56.600, 7.800)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        cornerIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.600, 7.800),

                                new Pose(11.122, 10.609)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.122, 10.609),

                                new Pose(11.028, 64.725)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(145))

                .build();

        scoreGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.028, 64.725),

                                new Pose(56.600, 7.800)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(140))

                .build();
    }


    public void buildRedPaths(){
        bottomSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.402, 7.598),

                                new Pose(102.827, 35.373)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                .build();

        Path8 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.827, 35.373),

                                new Pose(124.871, 35.620)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        scoreBottom = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.871, 35.620),

                                new Pose(83.672, 11.328)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        startToIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(83.672, 11.328),

                                new Pose(129.133, 10.910)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        farZoneScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.133, 10.910),

                                new Pose(83.672, 11.328)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        cornerIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(83.672, 11.328),

                                new Pose(128.805, 10.786)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.805, 10.786),

                                new Pose(132.972, 64.720)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(35))

                .build();

        scoreGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.972, 64.720),

                                new Pose(83.672, 11.328)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(40))

                .build();
    }
}


