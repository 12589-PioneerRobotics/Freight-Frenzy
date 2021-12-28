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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "trajectoryTest")
public class trajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory forwardTraj = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(34, 0))
                .build();
        Trajectory topRight = drive.trajectoryBuilder(forwardTraj.end())
                .lineToSplineHeading(new Pose2d(0, -20, Math.toRadians(-90)))
                .build();
        Trajectory bottomRight = drive.trajectoryBuilder(topRight.end())
                .lineToSplineHeading(new Pose2d(-34, 0, Math.toRadians(180)))
                .build();
        Trajectory bottomLeft = drive.trajectoryBuilder(bottomRight.end())
                .lineToSplineHeading(new Pose2d(0, 20, Math.toRadians(90)))
                .build();
        Trajectory topLeft = drive.trajectoryBuilder(bottomLeft.end())
                .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(0)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(forwardTraj);

        while(true) {
            drive.followTrajectory(topRight);
            drive.followTrajectory(bottomRight);
            drive.followTrajectory(bottomLeft);
            drive.followTrajectory(topLeft);
        }
    }
}
