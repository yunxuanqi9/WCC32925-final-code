package org.firstinspires.ftc.teamcode.oldFTC;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SimpleFlywheelPID;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "NextFTC Test")
public class PIDTest extends NextFTCOpMode {

    public ElapsedTime gateTimer = new ElapsedTime();
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
    public PIDTest() {
        addComponents(
                new SubsystemComponent(SimpleFlywheelPID.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void intakeSequencing(){
        if(!triggerRapidFire){
            SimpleFlywheelPID.INSTANCE.closeGate();
            Intake.INSTANCE.On.schedule();
        }
    }

    public void shooterSequencing() {
        gateTimer.reset();
        //Intake.INSTANCE.Off();
        SimpleFlywheelPID.INSTANCE.openGate();
        triggerRapidFire = true;

    }

    @Override public void onInit(){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shoot(){
        SimpleFlywheelPID.INSTANCE.On.schedule();
        telemetry.addData("works","ig");
        telemetry.update();
    }


    public static double upPower = 0.8;
    public static double downPower = 0.6;

    @Override
    public void onStartButtonPressed() {

        SimpleFlywheelPID.INSTANCE.closeGate();
        Gamepads.gamepad1().cross()
                .whenBecomesTrue(() -> shoot());
        Gamepads.gamepad1().circle()
                .whenBecomesTrue( SimpleFlywheelPID.INSTANCE.Off);
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> intakeSequencing())
                .whenBecomesFalse(Intake.INSTANCE.Off)
        ;
        Gamepads.gamepad1().square()
                .whenBecomesTrue(() -> {
                    shooterSequencing();
                });
        Gamepads.gamepad1().options()
                .whenBecomesTrue(Intake.INSTANCE.Unclog)
                .whenBecomesFalse(Intake.INSTANCE.Off);

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(SimpleFlywheelPID.INSTANCE.setSpeedHigh);

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(SimpleFlywheelPID.INSTANCE.setSpeedLow);
    }
    @Override
    public void onUpdate(){

        if(triggerRapidFire && gateTimer.seconds() > waitGate){
            Intake.INSTANCE.On.schedule();
            triggerRapidFire = false;
        }
        if(triggerRapidFire && gateTimer.seconds() > waitIntake){
            Intake.INSTANCE.Off.schedule();
            SimpleFlywheelPID.INSTANCE.Off.schedule();
        }
        telemetry.addData("flywheel velo",SimpleFlywheelPID.INSTANCE.Shooter1.getVelocity());
        telemetry.addData("gatetimer",gateTimer.seconds());
        telemetry.addData("flywheel velocity",SimpleFlywheelPID.INSTANCE.currSpeed);
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
        SimpleFlywheelPID.INSTANCE.closeGate();
    }
}
