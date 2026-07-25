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
import dev.nextftc.core.commands.groups.ParallelGroup;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


//@Autonomous(name = "Pedro Pathing 15 Ball Optimised", group = "Autonomous")
@Configurable // Panels
public abstract class Blue15Archive extends NextFTCOpMode {
    protected final boolean redTeam;
    protected final boolean isSolo;
    public Blue15Archive(Boolean redTeam, Boolean isSolo) {
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
                new FollowPath(middleSpike),
                new FollowPath(scoreMiddle),
                Intake.INSTANCE.Off,
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

                //TOP SPIKE MUST GO AT THE END!
                Intake.INSTANCE.On,
                new FollowPath(topSpike),
                new FollowPath(scoreTop),
                shootArtifacts(),
                new FollowPath(Leave)

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
    public PathChain awayGate;
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
                                new Pose(49.979, 59.404),
                                new Pose(20.900, 59.867)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))

                .build();

        scoreMiddle = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.900, 59.867),

                                new Pose(49.841, 92.882)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(49.841, 92.882),
                                new Pose(38.435, 56.148),
                                new Pose(11.736, 60.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(143))

                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.736, 60.446),
                                new Pose(18.606, 47.201),
                                new Pose(49.771, 92.989)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(49.771, 92.989),

                                new Pose(21.069, 83.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        scoreTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.069, 83.446),

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

                                new Pose(83.976, 92.963)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(-50))

                .build();

        middleSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.976, 92.963),
                                new Pose(94.021, 59.404),
                                new Pose(132.459, 57.897)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))

                .build();

        scoreMiddle = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.459, 57.897),

                                new Pose(83.886, 92.704)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        openGate = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.886, 92.704),
                                new Pose(105.565, 56.148),
                                new Pose(132.264, 60.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(37))

                .build();

        gateScore = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.264, 60.446),
                                new Pose(125.394, 47.201),
                                new Pose(83.779, 93.343)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        topSpike = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.779, 93.343),
                                new Pose(100.001, 82.383),
                                new Pose(125.411, 83.446)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        scoreTop = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.411, 83.446),

                                new Pose(102.457, 97.152)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Leave = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.457, 97.152),

                                new Pose(114.598, 60.753)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    public static void drawOnlyCurrent(){
        try{
            Drawing.drawRobot(follower().getPose());
            Drawing.sendPacket();
        } catch (Exception e){
            throw new RuntimeException("Drawing failed" + e);
        }
    }
}

