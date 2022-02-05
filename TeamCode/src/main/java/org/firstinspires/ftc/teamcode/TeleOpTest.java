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
            if(!slowMode) { // If the slowmode is toggled on, the robot will move twice as slow
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.right_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x)
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.right_stick_y / 2,
                                -gamepad1.right_stick_x / 2,
                                -gamepad1.left_stick_x)
                );
            }


       drive.update(); // Update SampleMecanumDrive

        // Print the input from the left stick
        telemetry.addData("X: ", gamepad1.right_stick_x);
        telemetry.addData("Y: ", gamepad1.right_stick_y);

        if(gamepadEvent1.rightStickButton()) { // If the right stick button is clicked down, toggle the slowmode
            slowMode = !slowMode;
        }

       if(gamepad1.right_trigger > 0.5){ // Run the intake when the right trigger is pressed down
           actuation.blockerOpen();
           actuation.intake();
       }
       else if(gamepad1.left_trigger > 0.5) { // Run the intake backwards when the left trigger is pressed down
           actuation.blockerClose();
           actuation.spitOut();
       }
       else { // If no triggers are pressed, stop the intake
           actuation.blockerClose();
           actuation.stopIntake();
       }

       if(gamepad2.right_bumper) // Open the depositor if the right bumper is pressed down
           actuation.depositorOpen();
       else // Close the depositor if the right bumper isn't pressed down
           actuation.depositorClose();


        if(gamepadEvent2.triangle()) { // Set the slides to the third position (highest)
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(3);
        }
        else if(gamepadEvent2.circle()) { // Set the slides to the second position (middle)
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(2);
        }
        else if(gamepadEvent2.cross()){ // Set the slides to the first position (bottom)
            actuation.setDepositorPosition(0.75);
            actuation.slideAction(1);
        }
        else if(gamepadEvent2.square()) // Reset the slides to the position for intake
            actuation.slideReset();


       if(gamepad2.right_trigger > 0.5) // Spin the carousel spinner right if the right trigger is pressed down
           actuation.carouselSpinRed();
        else if(gamepad2.left_trigger > 0.5) // Spins the carousel spinner left if the left trigger is pressed down
            actuation.carouselSpinBlue();
        else
            actuation.stopCarousel(); // Stop the carousel spinner if none of the buttons are pressed down

       telemetry.addData("Target Slide Position", targetSlidePosition); // Print the current target slide position
       telemetry.addData("Current Position: ", slide.getCurrentPosition()); // Print the actual current slide position

        // Print the current locations of all the wheels
       telemetry.addData("Front Left: ", drive.leftFront.getCurrentPosition());
       telemetry.addData("Front Right:", drive.rightFront.getCurrentPosition());
       telemetry.addData("Rear Left: ", drive.leftRear.getCurrentPosition());
       telemetry.addData("Rear Right: ", drive.rightRear.getCurrentPosition());
       telemetry.update();
    }
}
