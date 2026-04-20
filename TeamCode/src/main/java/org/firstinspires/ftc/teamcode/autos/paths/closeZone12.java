package org.firstinspires.ftc.teamcode.autos.paths;


import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitGate;
import static org.firstinspires.ftc.teamcode.nextFTCTeleOps.mainTeleOp.waitToKick;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;


import org.firstinspires.ftc.teamcode.robotConstants.Drawing;
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
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing 15 Ball Optimised", group = "Autonomous")
@Configurable // Panels
public abstract class closeZone12 extends NextFTCOpMode {

    protected final boolean redTeam;

    public closeZone12(Boolean redTeam) {
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

    private Pose startingPose = new Pose(19.192555751644484, 119.84007700946573, Math.toRadians(144));

    private Pose scoringPose = new Pose(65.1, 78.2);

    @Override
    public void onInit() {
        mainConstants.setAlliance(redTeam);
        Shooter.INSTANCE.Off.schedule();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        if (redTeam) {
            startingPose = startingPose.mirror(); // MIRRORS POSE!!!!
            scoringPose = scoringPose.mirror();
            buildRedPaths();

        } else {
            buildBluePaths();
        }

        follower().setStartingPose(startingPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.Init.schedule();
        Turret.INSTANCE.enableTracking.afterTime(0.01).schedule();

        autonomousRoutine().run();
    }


    public Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On,

                //MIDDLE SPIKE MUST GO FIRST!

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new InstantCommand(() -> follower().setMaxPower(0.7)),
                new FollowPath(middleSpike),
                Intake.INSTANCE.Off,
                new InstantCommand(() -> follower().setMaxPower(1)),
                new FollowPath(scoreMiddle),
                shootArtifacts(),


                //CAN REPEAT AS MANY AS YOU'D LIKE.

                new FollowPath(openGate).thenWait(mainConstants.holdGate),
                new FollowPath(awayGate),
                Intake.INSTANCE.On,
                new FollowPath(gateIntake),
                new FollowPath(gateScore),
                Intake.INSTANCE.Off,
                shootArtifacts(),

                //TOP SPIKE MUST GO AT THE END!

                Intake.INSTANCE.On,
                new InstantCommand(() -> follower().setMaxPower(0.7)),
                new FollowPath(topSpike),
                Intake.INSTANCE.Off,
                new InstantCommand(() -> follower().setMaxPower(1)),
                new FollowPath(scoreTop),
                shootArtifacts()
        );
    }

    @Override
    public void onUpdate() {
        // Log values to Panels and Driver Station

        Pose robotPose = follower().getPose();

        //Constantly edits

        if (robotPose.getX() != 0 && robotPose.getY() != 0 && robotPose.getHeading() != 0) {
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

        drawOnlyCurrent();
    }

    @Override
    public void onStop() {
        mainConstants.autoEndPose = follower().getPose();
        ActiveOpMode.telemetry().addData("End pose X", mainConstants.autoEndPose.getX());
        ActiveOpMode.telemetry().addData("End pose Y", mainConstants.autoEndPose.getY());
    }

    public Command shootArtifacts() {
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

    public PathChain shootPreload;
    public PathChain middleSpike;
    public PathChain scoreMiddle;
    public PathChain openGate;
    public PathChain awayGate;
    public PathChain gateIntake;
    public PathChain gateScore;
    public PathChain topSpike;
    public PathChain scoreTop;

    public void buildBluePaths() {
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.661, 119.486),

                                new Pose(30.090, 107.487)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(230))

                .build();

        middleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(30.090, 107.487),
                                new Pose(61.871, 76.547),
                                new Pose(36.872, 57.101),
                                new Pose(20.929, 59.846)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                .build();

        scoreMiddle = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.929, 59.846),

                                new Pose(50.200, 84.200)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.200, 84.200),

                                new Pose(15.720, 66.351)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))

                .build();

        awayGate = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.720, 66.351),

                                new Pose(26.520, 62.565)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .setReversed()
                .build();

        gateIntake = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.520, 62.565),

                                new Pose(10.269, 57.911)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))

                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.269, 57.911),

                                new Pose(50.200, 84.200)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.200, 84.200),

                                new Pose(18.589, 83.800)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        scoreTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.589, 83.800),

                                new Pose(36.229, 101.403)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void buildRedPaths() {
        
    }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower().getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }

    }
}

