package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.GamepadEventPS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class SlidesTest extends OpMode {
    SampleMecanumDrive drive;
    GamepadEventPS gamepadEvent;
    DcMotorEx slide;
    int targetPosition = 800;
    float servoPosition;

    Actuation actuation;

    Servo depositor;

    @Override
    public void init() {
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setTargetPosition(targetPosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.7);
        slide.setTargetPositionTolerance(5);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gamepadEvent = new GamepadEventPS(gamepad1);

        drive = new SampleMecanumDrive(hardwareMap);
        actuation = new Actuation(drive, null, this, hardwareMap);

        depositor = hardwareMap.get(Servo.class, "depositor");
        servoPosition = 1;

        telemetry.addLine("Initialized!");
    }

    @Override
    public void loop() {

        if(gamepadEvent.dPadUp())
            targetPosition += 100;
        else if(gamepadEvent.dPadDown())
            targetPosition -= 100;
        else if(gamepadEvent.rightBumper())
            slide.setTargetPosition(targetPosition);

        if(gamepadEvent.dPadLeft())
            servoPosition -= 0.05;
        else if(gamepadEvent.dPadRight())
            servoPosition += 0.05;

        depositor.setPosition(servoPosition);

        if(gamepad1.right_trigger > 0.5){
            actuation.blockerOpen();
            actuation.intake();
        }
        else if(gamepad1.left_trigger > 0.5) {
            actuation.blockerClose();
            actuation.spitOut();
        }
        else
            actuation.stopIntake();

        telemetry.addData("Target Position: ", targetPosition);
        telemetry.addData("Current Position: ", slide.getCurrentPosition());
        telemetry.addData("Target Depositor Position: ", servoPosition);
        telemetry.addData("Current Depositor Position: ", depositor.getPosition());
        telemetry.update();

    }
}
