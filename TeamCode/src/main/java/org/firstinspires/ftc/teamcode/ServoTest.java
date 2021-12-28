package org.firstinspires.ftc.teamcode;

import androidx.core.view.OneShotPreDrawListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.GamepadEventPS;

@TeleOp
public class ServoTest extends OpMode{


    Servo depositor, blocker;
    GamepadEventPS gamepadEvent;
    double depoPos = 0;
    double blockPos = 0;
    double increment = 0.05;



    public void init(){
        gamepadEvent = new GamepadEventPS(gamepad1);
        depositor = hardwareMap.get(Servo.class, "depositor");
        blocker = hardwareMap.get(Servo.class, "blocker");
        depositor.setPosition(0);
        blocker.setPosition(0);

        telemetry.addLine("Initialized!");
    }

    @Override
    public void loop(){
            if(gamepadEvent.dPadUp())
                depoPos += increment;
            if(gamepadEvent.dPadDown())
                depoPos -= increment;
            if (gamepadEvent.triangle())
                blockPos += increment;
            if(gamepadEvent.cross())
                blockPos -= increment;

            depositor.setPosition(depoPos);
            blocker.setPosition(blockPos);







            telemetry.addData("Depositor Position", depositor.getPosition());
            telemetry.addData("Blocker Position", blocker.getPosition());
            telemetry.update();
    }
}
