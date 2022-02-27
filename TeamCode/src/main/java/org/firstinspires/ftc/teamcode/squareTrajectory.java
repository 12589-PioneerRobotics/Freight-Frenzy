package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "squareTrajectory")
public class squareTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));//omkar is ugly
        drive.setPoseEstimate(startPose);

        Trajectory side1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(40, 20, Math.toRadians(90)))
                .build();

        Trajectory side2 = drive.trajectoryBuilder(side1.end())
                .lineToSplineHeading(new Pose2d(0, 40, Math.toRadians(180)))//omkar is ugly
                .build();
        /*Rushil is
        cute
        */


        Trajectory side3 = drive.trajectoryBuilder(side2.end())
                .lineToSplineHeading(new Pose2d(0, 0, Math.toRadians(-90)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(side1);
        drive.followTrajectory(side2);
        drive.followTrajectory(side3);

    }
}
