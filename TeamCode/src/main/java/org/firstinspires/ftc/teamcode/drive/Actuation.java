package org.firstinspires.ftc.teamcode.drive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Actuation {
    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    public DcMotorEx slide, intake, carousel;
    public Servo depositor, blocker;
    LinearOpMode linearOpMode;
    OpMode opMode;
    double slideHeightLimit;
    final double depositorClose = 0.70;
    final double depositorOpen = 0.3;
    final double blockerClose = 0.59;
    final double blockerOpen = 0.1;
    final double intakePower = 0.7;
    final double slidePower = 0.5;

    public Actuation(SampleMecanumDrive drive, LinearOpMode linearOpMode, OpMode opMode, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;

        if(hardwareMap.dcMotor.contains("intake")){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(hardwareMap.dcMotor.contains("slide")){
            slide = hardwareMap.get(DcMotorEx.class, "slide");
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(hardwareMap.dcMotor.contains("carousel")){
            carousel = hardwareMap.get(DcMotorEx.class, "carousel");
            carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    public void intake(){
        if(intake == null)
            return;
        intake.setPower(intakePower);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void spitOut(){
        if(intake == null)
            return;
        intake.setPower(-intakePower);
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

    public void stopSlide(){
        slide.setPower(0);
    }


}
