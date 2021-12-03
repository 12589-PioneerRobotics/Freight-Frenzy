package org.firstinspires.ftc.teamcode;

import androidx.core.view.OneShotPreDrawListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode{


    Servo depositor, blocker;
    double depoPos = 0;
    double blockPos = 0;
    double increment = 0.01;



    public void init(){
        depositor = hardwareMap.get(Servo.class, "depositor");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depositor.setPosition(0);
        blocker.setPosition(0);

        telemetry.addLine("Initialized!");
    }

    @Override
    public void loop(){
            if(gamepad1.dpad_up)
                depoPos += increment;
            if(gamepad1.dpad_down)
                depoPos -= increment;
            if (gamepad1.triangle)
                blockPos += increment;
            if(gamepad1.cross)
                blockPos -= increment;

            depositor.setPosition(depoPos);
            blocker.setPosition(blockPos);







            telemetry.addData("Depositor Position", depositor.getPosition());
            telemetry.addData("Blocker Position", blocker.getPosition());
            telemetry.update();
    }
}
