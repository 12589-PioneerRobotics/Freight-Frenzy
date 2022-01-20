package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.GamepadEventPS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp Test")
public class TeleOpTest extends OpMode {
    SampleMecanumDrive drive;
    Actuation actuation;
    GamepadEventPS gamepadEvent1, gamepadEvent2;
    DcMotorEx slide;
    int targetSlidePosition;
    int resetPosition;

    boolean depositorOpen;

    @Override
    public void init() {
        depositorOpen = false;
        targetSlidePosition = 1;
        resetPosition = -90; // Lower numbers mean less slack on the slides string (very unintuitive i know, but it works)
        gamepadEvent1 = new GamepadEventPS(gamepad1);
        gamepadEvent2 = new GamepadEventPS(gamepad2);
        drive = new SampleMecanumDrive(hardwareMap);
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setTargetPosition(targetSlidePosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.7);
        slide.setTargetPositionTolerance(5);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuation = new Actuation(drive, null, this, hardwareMap);
        telemetry.addLine("Initialized!");
    }

    @Override
    public void loop() {
       drive.setWeightedDrivePower(
               new Pose2d(
                       -gamepad1.right_stick_y,
                       -gamepad1.right_stick_x,
                       -gamepad1.left_stick_x)
       );

       drive.update();

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

       if(gamepadEvent1.rightBumper()) {
           if(!depositorOpen)
               actuation.depositorOpen();
           else actuation.setDepositorPosition(0.75);

           depositorOpen = !depositorOpen;
        }


        //
        setSlidePosition();
        if(gamepadEvent1.leftBumper()) {
            actuation.setDepositorPosition(0.75);
            slide.setTargetPosition(targetSlidePosition * 200);
        }
        if(gamepadEvent1.dPadLeft()) {
            actuation.depositorClose();
            slide.setTargetPosition(resetPosition);
        }

       if(gamepad2.right_trigger > 0.5)
           actuation.carouselSpinRed();
        else if(gamepad2.left_trigger > 0.5)
            actuation.carouselSpinBlue();
        else
            actuation.stopCarousel();

       telemetry.addData("Target Slide Position", targetSlidePosition);
       telemetry.addData("Current Position: ", slide.getCurrentPosition());
       telemetry.update();
    }

   private void setSlidePosition(){
        if(gamepadEvent1.dPadUp()){
            if(targetSlidePosition < 3)
                targetSlidePosition ++;
        }
        else if(gamepadEvent1.dPadDown()){
            if(targetSlidePosition > 1)
                targetSlidePosition --;
        }
    }



}
