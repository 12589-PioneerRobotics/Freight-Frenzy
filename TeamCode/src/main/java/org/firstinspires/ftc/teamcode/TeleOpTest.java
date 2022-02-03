package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    double thresh;

    boolean depositorOpen;
    boolean slowMode;

    @Override
    public void init() {
        depositorOpen = false;
        targetSlidePosition = 1;
        resetPosition = -20; // Lower numbers mean less slack on the slides string (very unintuitive i know, but it works)
        thresh = 0.01;
        gamepadEvent1 = new GamepadEventPS(gamepad1);
        gamepadEvent2 = new GamepadEventPS(gamepad2);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.HEADING_PID = new PIDCoefficients(0, 0, 0);
        drive.TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
        slowMode = false;
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
        //if(Math.abs(gamepad1.left_stick_x) > thresh || Math.abs(gamepad1.left_stick_y) > thresh) {
            //if(!slowMode) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.right_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x)
                );
//            }
//            else {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.right_stick_y / 2,
//                                -gamepad1.right_stick_x / 2,
//                                -gamepad1.left_stick_x)
//                );
//            }
        //}


       drive.update();

        telemetry.addData("X: ", gamepad1.right_stick_x);
        telemetry.addData("Y: ", gamepad1.right_stick_y);

        if(gamepadEvent1.rightStickButton()) {
            slowMode = !slowMode;
        }

       if(gamepad1.right_trigger > 0.5){
           actuation.blockerOpen();
           actuation.intake();
       }
       else if(gamepad1.left_trigger > 0.5) {
           actuation.blockerClose();
           actuation.spitOut();
       }
       else {
           actuation.blockerClose();
           actuation.stopIntake();
       }

       if(gamepad2.right_bumper)
           actuation.depositorOpen();
       else
           actuation.depositorClose();


        //
        setSlidePosition();
        if(gamepadEvent2.triangle()) {
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(3);
        }
        else if(gamepadEvent2.circle()) {
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(2);
        }
        else if(gamepadEvent2.cross()){
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(1);
        }
        else if(gamepadEvent2.square())
            actuation.slideReset();


       if(gamepad2.right_trigger > 0.5)
           actuation.carouselSpinRed();
        else if(gamepad2.left_trigger > 0.5)
            actuation.carouselSpinBlue();
        else
            actuation.stopCarousel();

       telemetry.addData("Target Slide Position", targetSlidePosition);
       telemetry.addData("Current Position: ", slide.getCurrentPosition());
       telemetry.addData("Front Left: ", drive.leftFront.getCurrentPosition());
       telemetry.addData("Front Right:", drive.rightFront.getCurrentPosition());
       telemetry.addData("Rear Left: ", drive.leftRear.getCurrentPosition());
       telemetry.addData("Rear Right: ", drive.rightRear.getCurrentPosition());
       telemetry.update();
    }

   private void setSlidePosition(){
        if(gamepadEvent2.dPadUp()){
            if(targetSlidePosition < 3)
                targetSlidePosition ++;
        }
        else if(gamepadEvent2.dPadDown()){
            if(targetSlidePosition > 1)
                targetSlidePosition --;
        }
    }



}
