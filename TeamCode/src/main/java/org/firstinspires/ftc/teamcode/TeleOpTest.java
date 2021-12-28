package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.GamepadEventPS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp Test")
public class TeleOpTest extends OpMode {
    SampleMecanumDrive drive;
    Actuation actuation;
    GamepadEventPS gamepadEvent1, gamepadEvent2;
    int slidePosition;


    @Override
    public void init() {
        slidePosition = 1;
        gamepadEvent1 = new GamepadEventPS(gamepad1);
        gamepadEvent2 = new GamepadEventPS(gamepad2);
        drive = new SampleMecanumDrive(hardwareMap);
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

       if(gamepad2.right_bumper)
           actuation.depositorOpen();
       else
           actuation.depositorClose();



       setSlidePosition();
       if(gamepadEvent2.cross())
           actuation.slideAction(1);
       else if(gamepadEvent2.circle())
           actuation.slideAction(2);
       else if(gamepadEvent2.triangle())
           actuation.slideAction(3);
       else if(gamepadEvent2.square())
           actuation.slideReset();

       if(gamepad2.right_trigger > 0.5)
           actuation.carouselSpinRed();
       else
           actuation.stopCarousel();

        if(gamepad2.left_trigger > 0.5)
            actuation.carouselSpinBlue();
        else
            actuation.stopCarousel();

       telemetry.addData("Target Slide Position", slidePosition);
       telemetry.update();
    }

    private void setSlidePosition(){
        if(gamepadEvent1.triangle()){
            if(slidePosition < 3)
                slidePosition++;
        }
        else if(gamepadEvent1.cross()){
            if(slidePosition > 1)
                slidePosition--;
        }
    }



}
