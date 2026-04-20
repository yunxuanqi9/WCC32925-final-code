package org.firstinspires.ftc.teamcode.nextFTCTeleOps;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.drawable.Drawable;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotConstants.Drawing;
import org.firstinspires.ftc.teamcode.robotConstants.mainConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
@TeleOp(name = "NextFTC Main TeleOp")
public class mainTeleOp extends NextFTCOpMode {

    private ControlSystem headingPIDF;

    DriverControlledCommand driverControlled;

    public static Pose startingPose =
            //new Pose(8.11,7.598,Math.toRadians(90));
            new Pose(72, 72, Math.toRadians(90));

    public static double waitGate = 1;
    public static double waitShoot = 1.2;
    public static double waitToKick = 0.4;

    private static double goalOffsetX = 0;
    private static double goalOffsetY = 0;

    public boolean triggerRapidFire = false;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    public static boolean headingLock = false;


    GamepadEx panelsGamePad;

    GamepadManager g1 = PanelsGamepad.INSTANCE.getFirstManager();

    public mainTeleOp() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(
                        //Lift.INSTANCE,
                        //newFlywheel.INSTANCE,
                        Shooter.INSTANCE,
                        Intake.INSTANCE,
                        Turret.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
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

    @Override
    public void onWaitForStart() {
    }

    @Override
    public void onInit() {

        if (mainConstants.autoEndX != 0 && mainConstants.autoEndY != 0 && mainConstants.autoEndHeading != 0) {
            startingPose = new Pose(mainConstants.autoEndX, mainConstants.autoEndY, mainConstants.autoEndHeading);
        }

        PedroComponent.follower().setPose(startingPose);

        headingPIDF = ControlSystem.builder()
                .posPid(1, 0, 0.04)
                .basicFF(0, 0, 0.025)
                .build();

        headingPIDF.setGoal(new KineticState(mainConstants.gateHeading));
    }

    @Override
    public void onStartButtonPressed() {

        driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                () -> {
                    if (headingLock) {
                        return headingPIDF.calculate(new KineticState(PedroComponent.follower().getHeading()));
                    } else {
                        return (double) gamepad1.right_stick_x * -1;
                    }
                },
                true
        );

        Shooter.INSTANCE.closeGate.schedule();
        Shooter.INSTANCE.Off.schedule();

        Turret.INSTANCE.enableTracking.schedule();
        //added delay


        PedroComponent.follower().startTeleOpDrive(true);

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Shooter.INSTANCE.On)
                .whenBecomesFalse(Shooter.INSTANCE.Off);

        Gamepads.gamepad1().triangle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> headingLock = true)
                .whenBecomesFalse(() -> headingLock = false);

        Gamepads.gamepad1().square()
                //UNCLOGGING FEATURE!!
                .whenBecomesTrue(
                        new ParallelGroup(
                                Shooter.INSTANCE.Off,
                                Shooter.INSTANCE.openGate,
                                Intake.INSTANCE.Unclog,
                                Shooter.INSTANCE.UnclogOn
                        )
                )
                .whenBecomesFalse(
                        new ParallelGroup(
                                Intake.INSTANCE.Off,
                                Shooter.INSTANCE.UnclogOff,
                                Shooter.INSTANCE.closeGate
                        )
                );

        Gamepads.gamepad1().rightTrigger().atLeast(0.3)
                .whenBecomesTrue(Intake.INSTANCE.intakeArtifacts)
                .whenBecomesFalse(Intake.INSTANCE.Off);

        Gamepads.gamepad1().leftTrigger().atLeast(0.4)
                .whenBecomesTrue(shootArtifacts()
                );

        Gamepads.gamepad1().ps().and(Gamepads.gamepad1().dpadUp()).whenBecomesTrue(() -> {
                            PedroComponent.follower().setPose(new Pose(72, 72, Math.toRadians(90)));});
        Gamepads.gamepad1().ps().and(Gamepads.gamepad1().dpadLeft())
                .whenBecomesTrue(() -> PedroComponent.follower().setPose(new Pose(mainConstants.robotWidth / 2, mainConstants.robotCenterDistBack, Math.toRadians(90))));
        Gamepads.gamepad1().ps().and(Gamepads.gamepad1().dpadRight()).whenBecomesTrue(() -> {
                    //bottom right corner facing top
                    PedroComponent.follower().setPose(new Pose(mainConstants.robotWidth / 2, mainConstants.robotCenterDistBack, Math.toRadians(90)).mirror());
                });
        Gamepads.gamepad1().ps().and(Gamepads.gamepad1().cross()).whenBecomesTrue(() -> {
                            //bottom right corner facing top
                            if (mainConstants.redTeam) {
                                //Set to RED goal corner.
                                PedroComponent.follower().setPose(new Pose(18.661190437991344, 119.48583346703032, Math.toRadians(144)
                                ).mirror());
                            } else {
                                //Set to BLUE goal corner.
                                PedroComponent.follower().setPose(new Pose(18.661190437991344, 119.48583346703032, Math.toRadians(144)
                                ));
                            }

                        });


        Gamepads.gamepad1().options()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Turret.INSTANCE.disableTracking)
                .whenBecomesFalse(Turret.INSTANCE.enableTracking);

        Gamepads.gamepad1().share()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Shooter.INSTANCE.disableVelo)
                .whenBecomesFalse(Shooter.INSTANCE.enableVelo);

        Gamepads.gamepad1().dpadUp()
                .and((Gamepads.gamepad1().ps().not())).whenBecomesFalse(Shooter.INSTANCE.increaseSpeedOffset);

        Gamepads.gamepad1().dpadDown()
                .and((Gamepads.gamepad1().ps().not())).whenBecomesFalse(Shooter.INSTANCE.decreaseSpeedOffset);

        Gamepads.gamepad1().dpadLeft()
                .and((Gamepads.gamepad1().ps().not())).whenBecomesFalse(Shooter.INSTANCE.increaseAngleOffset);

        Gamepads.gamepad1().dpadRight()
                .and((Gamepads.gamepad1().ps().not())).whenBecomesFalse(Shooter.INSTANCE.decreaseAngleOffset);


        //GAMEPAD2

        Gamepads.gamepad2().dpadUp()
                        .whenBecomesTrue(() -> mainConstants.increaseGoalOffsetY(2));
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(() -> mainConstants.increaseGoalOffsetY(-2));
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> mainConstants.increaseGoalOffsetX(+2));
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> mainConstants.increaseGoalOffsetX(-2));

        driverControlled.schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.update();
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("headinglock", headingLock);
        telemetry.addData("Red?", mainConstants.redTeam);

        drawOnlyCurrent();
    }

    public void onStop() {
        Shooter.INSTANCE.closeGate.schedule();
    }

    public static void drawOnlyCurrent(){
        try{
            Drawing.drawRobot(PedroComponent.follower().getPose());
            Drawing.sendPacket();
        } catch (Exception e){
            throw new RuntimeException("Drawing failed" + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(PedroComponent.follower());
    }
}