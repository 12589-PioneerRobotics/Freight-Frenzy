package org.firstinspires.ftc.teamcode.drive;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.HashMap;

public class Actuation {
    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    public DcMotorEx slide, intake;
    public CRServo carousel;
    public Servo depositor, blocker;
    public ColorSensor colorSensor;
    LinearOpMode linearOpMode;
    OpMode opMode;
    HashMap<Integer, Integer> slidePositionMap;

    final int slideCapping = 800;
    final int slideHeightLimit = 700;
    final int slideMidPos = 400;
    final int slideBottomPos = 300;
    final int slideInitPos = 0;
    public final double depositorClose = 0.79;
    public final double depositorOpen = 0.425;
    public final double depositorCap = 0.385;
    final double blockerClose = 0.50;
    final double blockerOpen = 0.05;
    final double intakeVelocity = 2000.0;
    final double slidePower = 0.70;
    final double carouselPower = 0.5;
    final double intakePower = 0.7;
    public final double ticksPerRev = 384.5;


    public Actuation(SampleMecanumDrive drive, LinearOpMode linearOpMode, OpMode opMode, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.drive = drive;
        this.linearOpMode = linearOpMode;
        this.opMode = opMode;
        slidePositionMap = new HashMap<>(4);
        slidePositionMap.put(1, slideBottomPos);
        slidePositionMap.put(2, slideMidPos);
        slidePositionMap.put(3, slideHeightLimit);
        slidePositionMap.put(4, slideCapping);

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

        if(hardwareMap.crservo.contains("carousel")){
            carousel = hardwareMap.get(CRServo.class, "carousel");
        }

        if(hardwareMap.servo.contains("depositor")){
            depositor = hardwareMap.get(Servo.class, "depositor");
            depositor.setPosition(depositorClose);
        }

        if(hardwareMap.servo.contains("blocker")){
            blocker = hardwareMap.get(Servo.class, "blocker");
            blocker.setPosition(blockerOpen);
        }

        if(hardwareMap.colorSensor.contains("colorSensor")) {
            colorSensor = hardwareMap.colorSensor.get("colorSensor");
            colorSensor.enableLed(true);
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

    public void depositorCapstone() {
        if(depositor == null)
            return;
        depositor.setPosition(depositorCap);
    }

    public void intake(){
        if(intake == null)
            return;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setVelocity(-intakeVelocity);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void spitOut(){
        if(intake == null)
            return;
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        carousel.setPower(carouselPower);
    }

    public void carouselSpinBlue(){
        if(carousel == null)
            return;
        carousel.setPower(-carouselPower);
    }

    public void stopCarousel(){
        if(carousel == null)
            return;
        carousel.setPower(0.0);
    }

    public void intakeReset(){
        intake.setTargetPositionTolerance(10);
        intake.setTargetPosition((int) ((Math.ceil((double) intake.getCurrentPosition() / ticksPerRev)) * ticksPerRev));
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
