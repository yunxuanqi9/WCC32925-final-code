package org.firstinspires.ftc.teamcode.NextFTC;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dodgyLastMinute.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "NextFTC TeleOp Program Java")
public class TeleOpProgram extends NextFTCOpMode {

    /*public ElapsedTime gateTimer = new ElapsedTime();
    private Follower follower;
    public static Pose startingPose = new Pose(56,8,Math.toRadians(90));
    public static double setHood = 36;
    public static double waitGate = 1;
    public static double waitIntake = 2;

    public double setTurretAngle = 200;

    public boolean triggerRapidFire = false;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    public TeleOpProgram() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void intakeSequencing(){
        if(!triggerRapidFire){
            Flywheel.INSTANCE.closeGate();
            Intake.INSTANCE.On();
        }
    }

    public void shooterSequencing() {
        gateTimer.reset();
        Intake.INSTANCE.Off();
        Flywheel.INSTANCE.openGate();
        triggerRapidFire = true;

    }

    @Override public void onInit(){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot(){
        Flywheel.INSTANCE.On();
        telemetry.addData("works","ig");
        telemetry.update();
    }


    public static double upPower = 0.65;
    public static double downPower = 0.45;

    @Override
    public void onStartButtonPressed() {

        Flywheel.INSTANCE.closeGate();
        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> shoot());
        Gamepads.gamepad1().circle()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.Off());
        Gamepads.gamepad1().triangle()
                .whenBecomesTrue(() -> intakeSequencing())
                .whenBecomesFalse(() -> Intake.INSTANCE.Off());
        Gamepads.gamepad1().square()
                .whenBecomesTrue(() -> {
                    shooterSequencing();
                });
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.setTurretAngle(150));

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.setFlywheelPower(upPower));

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.setFlywheelPower(downPower));
    }
    @Override
    public void onUpdate(){
        //Flywheel.INSTANCE.autoFlywheelPower(follower);

        if(triggerRapidFire && gateTimer.seconds() > waitGate){
            Intake.INSTANCE.rapidFire();
            triggerRapidFire = false;
        }
        if(triggerRapidFire && gateTimer.seconds() > waitIntake){
            Intake.INSTANCE.Off();
            Flywheel.INSTANCE.Off();
        }
        follower.update();
        Flywheel.INSTANCE.setHoodAngle(50);
        telemetry.addData("hood pos",Flywheel.INSTANCE.hoodPos());
        telemetry.addData("robot x",follower.getPose().getX());
        telemetry.addData("robot y",follower.getPose().getY());
        telemetry.addData("robot velo",follower.getVelocity().getMagnitude());
        telemetry.addData("turret pos",Flywheel.INSTANCE.getPos());
        telemetry.addData("flywheel velo",Flywheel.INSTANCE.Shooter1.getVelocity());
        telemetry.addData("gatetimer",gateTimer.seconds());
        telemetry.addData("flywheel velocity",Flywheel.INSTANCE.flywheelPower);

        telemetry.update();


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }

    public void onStop(){
        Flywheel.INSTANCE.closeGate();
    }*/
}