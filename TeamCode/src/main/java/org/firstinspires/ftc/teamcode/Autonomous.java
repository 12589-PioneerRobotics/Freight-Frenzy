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
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Field;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode{
    Actuation actuation;
    SampleMecanumDrive drive;
    Pipeline pipeline;
    int cameraViewId;
    WebcamName webcamName;
    OpenCvCamera camera;
    FieldConstants.ShippingElementPosition elementPosition;
    int slidePosition;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap); // Initializes roadrunner
        actuation = new Actuation(drive, this, null, hardwareMap); // Initializes the actuation class
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // sets the ID for the camera
        pipeline = new Pipeline(); // Initializes the pipeline for the camera
        webcamName = hardwareMap.get(WebcamName.class, "webcam1"); // Gets the webcam from the hardware map
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId); // Initializes the camera
        camera.setPipeline(pipeline); // Sets the cameras pipeline
        drive.setPoseEstimate(FieldConstants.redCarouselStart); // Sets the starting position for the robot
        Trajectory toCarousel = drive.trajectoryBuilder(FieldConstants.redCarouselStart) // Build the trajectory to move from the starting position to the carousel
                .lineToLinearHeading(FieldConstants.redCarousel)
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
/*
        sleep(100);
        elementPosition = pipeline.getDetectionResults();
        sleep(200);
        elementPosition = pipeline.getDetectionResults();

        elementPosition = pipeline.getDetectionResults();

        if (elementPosition == FieldConstants.ShippingElementPosition.LEFT)
            slidePosition = 1;
        else if (elementPosition == FieldConstants.ShippingElementPosition.CENTER)
            slidePosition = 2;
        else if (elementPosition == FieldConstants.ShippingElementPosition.RIGHT)
            slidePosition = 3;


 */
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
        sleep(5000);
        drive.followTrajectory(toCarousel); // move to carousel

        actuation.carouselSpinRed(); // Spin the carousel spinner for the red side
        sleep(2000);
        actuation.stopCarousel(); // Stop the carousel 
        sleep(200);

        Trajectory transition = drive.trajectoryBuilder(drive.getPoseEstimate()) // Builds the default case for the transition point trajectory
                .lineToLinearHeading(new Pose2d(FieldConstants.transitionPoint.getX(), FieldConstants.redShippingHub.getY(), Math.toRadians(180)))
                .build();
        Trajectory toHub = drive.trajectoryBuilder(transition.end()) // Builds the trajectory from the transition point to the shipping hub
                .lineToConstantHeading(new Vector2d(FieldConstants.redShippingHub.getX() - 23, FieldConstants.redShippingHub.getY() + 6))
                .build();
        switch (elementPosition) { // change transition point based on the position of the capstone (from cv) (so it doesnt displace it)
            case CENTER:
                break;
            case LEFT:
                toHub = drive.trajectoryBuilder(transition.end())
                        .lineToConstantHeading(new Vector2d(FieldConstants.redShippingHub.getX() - 24, FieldConstants.redShippingHub.getY() + 6))
                        .build();
                break;
            case RIGHT:
                toHub = drive.trajectoryBuilder(transition.end())
                        .lineToConstantHeading(new Vector2d(FieldConstants.redShippingHub.getX() - 21, FieldConstants.redShippingHub.getY() + 6))
                        .build();
                break;
        }

        drive.followTrajectory(transition); // Follow the trajectory to a transition point that allows the robot to avoid the capstone
        sleep(200);
        drive.followTrajectory(toHub); // Drive from the transition point to the shipping hub
        actuation.slideAction(slidePosition); // Move the slide to the position corresponding to the capstone 
        sleep(500);
        actuation.depositorOpen(); // Open the depositor
        sleep(1500);
        actuation.depositorClose(); // Close the depositor
        sleep(200);

        Trajectory toDepot = drive.trajectoryBuilder(toHub.end()) // Build the trajectory to the depot from the shipping hub position
                .lineToSplineHeading(new Pose2d(FieldConstants.transitionPoint2.getX(), FieldConstants.transitionPoint2.getY(), Math.toRadians(0)))
                .build();

        Trajectory transition2 = drive.trajectoryBuilder(toDepot.end()) // A transition point from the depot to the warehouse to avoid hitting the capstone
                .strafeTo(FieldConstants.transitionPoint3)
                .build();

        Trajectory toPark = drive.trajectoryBuilder(transition2.end()) // Build the trajectory from the transition point to the parking spot
                .splineToConstantHeading(FieldConstants.evasiveRed, Math.toRadians(0))
                .splineToConstantHeading(FieldConstants.redWarehouse, Math.toRadians(0))
                .build();

        actuation.slideReset(); // Reset the slide position
        drive.followTrajectory(toDepot); // Follow the trajectory to the depot
        drive.followTrajectory(transition2); // Move to the transition point
        drive.followTrajectory(toPark); // Move to the parking spot and park
    }


}

class Pipeline extends OpenCvPipeline {


    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point topAnchor1 = new Point(10, 160);
    static final Point topAnchor2 = new Point(270, 160);
    static final Point bottomAnchor1 = new Point(topAnchor1.x + 50, topAnchor1.y + 50);
    static final Point bottomAnchor2 = new Point(topAnchor2.x + 50, topAnchor2.y + 50);

    FieldConstants.ShippingElementPosition position = FieldConstants.ShippingElementPosition.LEFT;
    Mat hls = new Mat();
    Mat region1, region2;
    public double averageH1, averageH2, averageS1, averageS2;


    @Override
    public void init(Mat mat) {

        Imgproc.cvtColor(mat, hls, Imgproc.COLOR_RGB2HLS);
        region1 = hls.submat(new Rect(topAnchor1, bottomAnchor1));
        region2 = hls.submat(new Rect(topAnchor2, bottomAnchor2));

    }


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hls, Imgproc.COLOR_RGB2HLS);

        // Averages the hue and saturation
        averageH1 = Core.mean(region1).val[0];
        averageH2 = Core.mean(region2).val[0];
        averageS1 = Core.mean(region1).val[2];
        averageS2 = Core.mean(region2).val[2];


        // Draws rectanglular regions to be scanned for the capstone
        Imgproc.rectangle(input, topAnchor1, bottomAnchor1, BLUE, 1);
        Imgproc.rectangle(input, topAnchor2, bottomAnchor2, BLUE, 1);

        // takes average hue, and/or saturation to detect where the capstone is located 
        if(averageS1 > 70 && averageS2 > 70){
            if(averageH1 > 60 && averageH1 < 75)
                position = FieldConstants.ShippingElementPosition.LEFT;
            else if(averageH2 > 60 && averageH2 < 75) {
                position = FieldConstants.ShippingElementPosition.CENTER;
            }
            else
                position = FieldConstants.ShippingElementPosition.RIGHT;
        }

        else if(averageS1 > 70)
            position = FieldConstants.ShippingElementPosition.LEFT;
        else if(averageS2 > 70) {
            position = FieldConstants.ShippingElementPosition.CENTER;
        }
        else
            position = FieldConstants.ShippingElementPosition.RIGHT;



        return input;

    }

    public FieldConstants.ShippingElementPosition getDetectionResults() { return position; }

}
