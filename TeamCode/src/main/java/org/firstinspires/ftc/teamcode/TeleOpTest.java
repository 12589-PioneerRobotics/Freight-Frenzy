package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp Test")
public class TeleOpTest extends LinearOpMode {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public DcMotor intake = null;
    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight  = hardwareMap.get(DcMotor.class, "front_right");
        backLeft  = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection((DcMotorSimple.Direction.REVERSE));

        double xInput;
        double yInput;
        double turnInput;

        double leftTrigger;
        double rightTrigger;

        waitForStart();
        while(opModeIsActive()) {
            xInput = gamepad1.right_stick_x;
            yInput = gamepad1.right_stick_y;

            turnInput = gamepad1.left_stick_x;

            leftTrigger = gamepad1.left_trigger;
            rightTrigger = gamepad1.right_trigger;

            libertyDrive(yInput, turnInput, xInput);

            if(leftTrigger > rightTrigger)
                intake.setPower(-leftTrigger);
            else if(rightTrigger > leftTrigger)
                intake.setPower(rightTrigger);
        }
    }

    public void libertyDrive(double drive, double turn, double strafe) {
        double factor = Math.abs(drive) + Math.abs(turn) + Math.abs(strafe);
        if (factor <= 1)
            factor = 1;
        backLeft.setPower((drive - strafe + turn) / factor);
        frontLeft.setPower((drive + strafe + turn) / factor);
        backRight.setPower((drive + strafe - turn) / factor);
        frontRight.setPower((drive - strafe - turn) / factor);
    }
}
