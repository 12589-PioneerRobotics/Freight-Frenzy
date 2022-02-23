package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.GamepadEventPS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Grabber Test")
public class grabberTest extends OpMode {
    SampleMecanumDrive drive;
    Actuation actuation;
    GamepadEventPS gamepadEvent1, gamepadEvent2;
    boolean slowMode;
    double armPos, clawPos = 0.0;
    /*
    double clawPos = 0.2;
    double armPos, loweredClawPos = 0.0;
    */

    @Override
    public void init() {
        gamepadEvent1 = new GamepadEventPS(gamepad1);
        gamepadEvent2 = new GamepadEventPS(gamepad2);
        drive = new SampleMecanumDrive(hardwareMap);
        actuation = new Actuation(drive, null, this, hardwareMap);
        slowMode = false;
        telemetry.addLine("Initialized!");
    }

    @Override
    public void loop() {
        if(gamepadEvent1.dPadUp())
            armPos += 0.05;
        if(gamepadEvent1.dPadDown())
            armPos -= 0.05;

        if(gamepadEvent1.dPadLeft())
            clawPos -= 0.1;
        if(gamepadEvent1.dPadRight())
            clawPos += 0.1;

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

/*
        if(gamepad1.square)
            actuation.setArmPosition(1.00);
*/
        /*
        if(gamepadEvent1.leftBumper())
            actuation.grabberClaw.setPosition(clawPos);
        */

        /*
        if (armPos >= 0.75 && actuation.clawIdle) {
            actuation.grabberClaw.setPosition(loweredClawPos);
        }
        */

    }
}
