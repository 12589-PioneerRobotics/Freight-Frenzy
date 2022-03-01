package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;//omkar is pretty
import org.openftc.easyopencv.OpenCvPipeline;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousWarehouse extends LinearOpMode {
    Actuation actuation;
    SampleMecanumDrive drive;
    Pipeline pipeline;
    int cameraViewId;
    WebcamName webcamName;
    OpenCvCamera camera;
    FieldConstants.ShippingElementPosition elementPosition;
    int slidePosition;//omkar is ugly
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap); // Initializes roadrunner
        actuation = new Actuation(drive, this, null, hardwareMap); // Initializes the actuation class
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // sets the ID for the camera
        pipeline = new Pipeline(); // Initializes the pipeline for the camera
        webcamName = hardwareMap.get(WebcamName.class, "webcam1"); // Gets the webcam from the hardware map
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId); // Initializes the camera
        camera.setPipeline(pipeline); // Sets the cameras pipeline
        drive.setPoseEstimate(FieldConstants.redWarehouseStart); // Sets the starting position for the robot
        Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(FieldConstants.redShippingHub.getX() + 6, FieldConstants.redShippingHub.getY() - 19, Math.toRadians(-60)))
                .build();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { // When the camera is opened
                //testCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT); // Start streaming the camera for use
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("Element Position: ", pipeline.getDetectionResults()); // Prints where the capstone is located (from CV)
        telemetry.addLine("Waiting for start"); // waiting for start
        telemetry.update(); // Update the telemetry

        waitForStart(); // Wait for the start

        sleep(100);

        elementPosition = pipeline.getDetectionResults(); // Gets the detection results from the camera

        switch(elementPosition){ // Sets the slide postions based on the position of the capstone (from CV)
            case LEFT:
                slidePosition = 1;
                break;
            case CENTER:
                slidePosition = 2;
                break;
            case RIGHT:
                slidePosition = 3;
                break;
            default:
                slidePosition = 3;
                break;
        }


        telemetry.addData("Element Position: ", elementPosition); // Prints the element position
        telemetry.addData("SAverage 1: ", pipeline.averageS1); // Prints the saturation average for the first box
        telemetry.addData("SAverage 2: ", pipeline.averageS2); // Prints the saturation average for the second box
        telemetry.addData("HAverage 1: ", pipeline.averageH1); // Prints the hue average for the first box
        telemetry.addData("HAverage 2: ", pipeline.averageH2); // Prints the hue average for the second box
        telemetry.update(); // Updates the telemetry




        camera.closeCameraDevice(); // Close Camera
        switch (elementPosition) {
            case CENTER:
                break;
            case LEFT:
                toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(FieldConstants.redShippingHub.getX() + 6, FieldConstants.redShippingHub.getY() - 20.5, Math.toRadians(-60)))
                        .build();
                break;
            case RIGHT:
                toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(FieldConstants.redShippingHub.getX() + 5, FieldConstants.redShippingHub.getY() - 15, Math.toRadians(-60)))
                        .build();
        }

        drive.followTrajectory(toHub);
        sleep(200);
        actuation.slideAction(slidePosition);
        sleep(500);
        actuation.depositorOpen(); // Open the depositor
        sleep(1500);
        actuation.depositorClose(); // Close the depositor
        sleep(500);
        actuation.slideReset();

        Trajectory transition = drive.trajectoryBuilder(toHub.end())
                .lineToSplineHeading(new Pose2d(FieldConstants.warehouseTransition.getX(), FieldConstants.warehouseTransition.getY(), Math.toRadians(0)))
                .build();

        drive.followTrajectory(transition);

        Trajectory toWarehouse = drive.trajectoryBuilder(transition.end())
                .lineToSplineHeading(new Pose2d(FieldConstants.redWarehouse.getX() + 16.5, FieldConstants.redWarehouse.getY() - 30, Math.toRadians(-30)))
                .build();

        drive.followTrajectory(toWarehouse);

        while (!(actuation.colorSensor.green() > 600)) {
            actuation.intake();
        }

        actuation.blockerClose();
        sleep(500);
        actuation.spitOut();
        sleep( 1000);
        actuation.stopIntake();
        sleep(300); //sleep after intake

        Trajectory transition2 = drive.trajectoryBuilder(toWarehouse.end())// omkar is ugly
                .lineToSplineHeading(new Pose2d(FieldConstants.warehouseTransition2.getX(), FieldConstants.warehouseTransition2.getY(), Math.toRadians(0)))
                .build();

        Trajectory transition3 = drive.trajectoryBuilder(transition2.end())//omkar is ugly
                .lineToSplineHeading(new Pose2d(FieldConstants.warehouseTransition.getX() + 20, FieldConstants.warehouseTransition.getY() + 5, Math.toRadians(0)))
                .build();

        Trajectory toHub2 = drive.trajectoryBuilder(transition3.end())
                .lineToSplineHeading(new Pose2d(FieldConstants.redShippingHub.getX() + 12, FieldConstants.redShippingHub.getY() - 14, Math.toRadians(-60)))
                .build();

        Trajectory toPark = drive.trajectoryBuilder(transition.end())
                .lineToConstantHeading(new Vector2d(FieldConstants.redWarehouse.getX() + 10, FieldConstants.redWarehouse.getY() - 15))
                .build();

        drive.followTrajectory(transition2);
        drive.followTrajectory(transition3);
        sleep(500);
        drive.followTrajectory(toHub2);
        sleep(200);
        actuation.slideAction(3);
        sleep(500);
        actuation.depositorOpen(); // Open the depositor
        sleep(1500);
        actuation.depositorClose(); // Close the depositor
        sleep(500);
        actuation.slideReset();
        drive.followTrajectory(transition);
        drive.followTrajectory(toPark);

    }
}
