package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOp Test")
public class TeleOpTest extends OpMode {
    SampleMecanumDrive drive;
    Actuation actuation;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        actuation = new Actuation(drive, null, this, hardwareMap);
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
       else
           actuation.stopIntake();


       if(gamepad1.left_trigger > 0.5) {
           actuation.blockerClose();
           actuation.spitOut();
       }
       else
           actuation.stopIntake();

       if(gamepad1.dpad_up)
           actuation.depositorOpen();
       if(gamepad1.dpad_down)
           actuation.depositorClose();

       if(gamepad1.triangle)
           actuation.slideUp();


       if(gamepad1.cross)
           actuation.slideDown();





    }

}
