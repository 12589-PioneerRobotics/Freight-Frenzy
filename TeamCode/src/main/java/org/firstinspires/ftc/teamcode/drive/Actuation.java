package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Actuation {
    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    public DcMotorEx slide, intake, carousel;
    public Servo depositor, blocker, grabberArm, grabberClaw;
    LinearOpMode linearOpMode;
    OpMode opMode;
    HashMap<Integer, Integer> slidePositionMap;

    final int slideHeightLimit = 700;
    final int slideMidPos = 400;
    final int slideBottomPos = 200;
    final int slideInitPos = 0;
    public final double depositorClose = 0.82;
    public final double depositorOpen = 0.40;
    final double blockerClose = 0.50;
    final double blockerOpen = 0.05;
    final double intakeVelocity = 2000.0;
    final double slidePower = 0.70;
    final double carouselPower = 0.80;
    final double intakePower = 0.7;
    final double clawClosed = 0.0;
    final double clawOpen = 0.9;
    final double armDown = 0.0;
    final double armUp = 0.4;
    public boolean clawIdle = true;

    public Actuation(SampleMecanumDrive drive, LinearOpMode linearOpMode, OpMode opMode, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;
        slidePositionMap = new HashMap<>(3);
        slidePositionMap.put(1, slideBottomPos);
        slidePositionMap.put(2, slideMidPos);
        slidePositionMap.put(3, slideHeightLimit);

        if(hardwareMap.dcMotor.contains("intake")){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(hardwareMap.dcMotor.contains("slide")){
            slide = hardwareMap.get(DcMotorEx.class, "slide");
            slide.setTargetPosition(slideInitPos);
            slide.setTargetPositionTolerance(5);
            slide.setPower(slidePower);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        if(hardwareMap.dcMotor.contains("carousel")){
            carousel = hardwareMap.get(DcMotorEx.class, "carousel");
            carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(hardwareMap.servo.contains("depositor")){
            depositor = hardwareMap.get(Servo.class, "depositor");
            depositor.setPosition(depositorClose);
        }

        if(hardwareMap.servo.contains("blocker")){
            blocker = hardwareMap.get(Servo.class, "blocker");
            blocker.setPosition(blockerOpen);
        }

        if(hardwareMap.servo.contains("grabberArm")) {
            grabberArm = hardwareMap.get(Servo.class, "grabberArm");
            grabberArm.setPosition(0.0);
        }

        if(hardwareMap.servo.contains("grabberClaw")) {
            grabberClaw = hardwareMap.servo.get("grabberClaw");
            grabberClaw.setPosition(clawClosed);
        }
    }

    public void blockerOpen(){
        if(blocker == null)
            return;
        blocker.setPosition(blockerOpen);
    }

    public void blockerClose(){
        if(blocker == null)
            return;
        blocker.setPosition(blockerClose);
    }

    public void depositorOpen(){
        if(depositor == null)
            return;
        depositor.setPosition(depositorOpen);
    }

    public void depositorClose(){
        if(depositor == null)
            return;
        depositor.setPosition(depositorClose);
    }

    public void setDepositorPosition(double position) {
        if(depositor == null)
            return;
        depositor.setPosition(position);
    }

    public void intake(){
        if(intake == null)
            return;
        intake.setVelocity(-intakeVelocity);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void spitOut(){
        if(intake == null)
            return;
        intake.setVelocity(intakeVelocity);
    }

    public void slideUp(){
        if(slide == null || slide.getCurrentPosition() >= 5000)
            return;
        slide.setPower(slidePower);
    }

    public void slideDown(){
        if(slide == null || slide.getCurrentPosition() < 10)
            return;
        slide.setPower(-slidePower);
    }

//    public void stopSlide(){
//        slide.setPower(0);
//    }

    public void slideAction(int position){
        if(slide == null)
            return;
        slide.setTargetPosition(slidePositionMap.get(position));
    }

    public void slideReset(){
        if(slide == null)
            return;
        slide.setTargetPosition(slideInitPos);
    }

    public void carouselSpinRed(){
        if(carousel == null)
            return;
        carousel.setPower(0.5);
    }

    public void carouselSpinBlue(){
        if(carousel == null)
            return;
        carousel.setPower(-0.7);
    }

    public void stopCarousel(){
        if(carousel == null)
            return;
        carousel.setPower(0);
    }

    public void setArmPosition(double pos){
        grabberArm.setPosition(pos);
    }

    public void clawAction(){
        if(grabberClaw.getPosition() < clawOpen) {
            clawIdle = false;
            grabberClaw.setPosition(clawOpen);
        }
        else {
            grabberClaw.setPosition(clawClosed);
            clawIdle = true;
        }
    }

}
