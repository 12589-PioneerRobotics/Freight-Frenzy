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
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory forwardTraj = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(30)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(forwardTraj);

    }
}
