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
import dev.nextftc.core.commands.groups.ParallelGroup;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Pedro Pathing 15 Ball Optimised", group = "Autonomous")
@Configurable // Panels
public abstract class Red12BallPath extends NextFTCOpMode {
    protected final boolean redTeam;
    protected final boolean isSolo;
    public Red12BallPath(Boolean redTeam, Boolean isSolo) {
        addComponents(
                new SubsystemComponent(
                        Turret.INSTANCE,
                        Intake.INSTANCE,
                        Shooter.INSTANCE
                ),
                new PedroComponent(Constants::createFollower)
        );
        this.redTeam = redTeam;
        this.isSolo = isSolo;
        mainConstants.redTeam = redTeam;
    }
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Pose startingPose = new Pose(19.015, 119.663, Math.toRadians(144));
    private Pose openGatePose = mainConstants.gateIntake;
    private Pose scoringPose = new Pose(65.1, 78.2);

    private double intakeWait = 1;
    private double spinUp = 0.1;

    @Override
    public void onInit() {
        mainConstants.setAlliance(redTeam);
        Shooter.INSTANCE.Off.schedule();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        if (redTeam) {
            startingPose = startingPose.mirror(); // MIRRORS POSE!!!!
            openGatePose = openGatePose.mirror();
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
        if(isSolo){
            SoloRoutine().run();
        }
        else{
            teamRoutine().run();
        }
    }

    public Command teamRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On.thenWait(spinUp),
                //MIDDLE SPIKE MUST GO FIRST!

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(middleSpike),
                Intake.INSTANCE.Off,
                new FollowPath(scoreMiddle),
                shootArtifacts(),

                //CAN REPEAT AS MANY AS YOU'D LIKE.
                Intake.INSTANCE.On,
                new FollowPath(openGate).thenWait(mainConstants.waitGateIntake),
                new FollowPath(gateScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(openGate).thenWait(mainConstants.waitGateIntake),
                new FollowPath(gateScore),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(openGate).thenWait(mainConstants.waitGateIntake),
                new FollowPath(gateScore),
                shootArtifacts(),

                //TOP SPIKE MUST GO AT THE END!
                Intake.INSTANCE.On,
                new FollowPath(topSpike),
                Intake.INSTANCE.Off

        );
    }


    public Command SoloRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.On.thenWait(spinUp),
                //MIDDLE SPIKE MUST GO FIRST!

                new FollowPath(shootPreload),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(topSpike),
                new FollowPath(scoreTop),
                shootArtifacts(),

                Intake.INSTANCE.On,
                new FollowPath(middleSpike),
                new FollowPath(Path7),
                Intake.INSTANCE.Off,
                new FollowPath(scoreMiddle),
                shootArtifacts(),

                new FollowPath(Leave)

                //CAN REPEAT AS MANY AS YOU'D LIK
                //TOP SPIKE MUST GO AT THE END!

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
    }

    @Override
    public void onStop() {
        mainConstants.autoEndPose = follower().getPose();
        Shooter.INSTANCE.Off.schedule();
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
    public PathChain gateScore;
    public PathChain Path7;
    public PathChain topSpike;
    public PathChain scoreTop;
    public PathChain Leave;

    public void buildBluePaths() {
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.015, 119.663),

                                new Pose(49.220, 84.284)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(230))

                .build();

        middleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(49.220, 84.284),
                                new Pose(51.396, 62.061),
                                new Pose(14.552, 63.742)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))

                .build();

        scoreMiddle = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.49,  0, 61.440),

                                new Pose(50.372, 82.608)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.372, 82.608),
                                new Pose(38.435, 56.148),
                                openGatePose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(143))

                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierLine(
                                openGatePose,

                                new Pose(50.480, 82.539)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.480, 82.539),

                                new Pose(13.984, 85.040)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        scoreTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.984, 85.040),

                                new Pose(41.543, 97.152)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Leave = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(41.543, 97.152),

                                new Pose(29.402, 60.753)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
    public void buildRedPaths(){
        shootPreload = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(124.630, 120.017),

                                new Pose(72.640, 85.346)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(-50))

                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(72.640, 85.346),
                                new Pose(100.001, 82.383),
                                new Pose(125.411, 83.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        scoreTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.411, 83.446),

                                new Pose(72.443, 85.018)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        middleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(72.443, 85.018),
                                new Pose(94.021, 59.404),
                                new Pose(132.459, 57.897)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();

        Path7 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.459, 57.897),

                                new Pose(126.487, 72.244)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))

                .build();

        scoreMiddle = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.487, 72.244),


                                new Pose(75.385, 73.044)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Leave = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(75.385, 73.044),

                                new Pose(102.376, 72.974)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}

