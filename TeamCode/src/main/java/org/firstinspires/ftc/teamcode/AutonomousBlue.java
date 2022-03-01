package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Actuation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousBlue extends LinearOpMode{
    Actuation actuation;
    SampleMecanumDrive drive;
    Pipeline pipeline;
    int cameraViewId;
    WebcamName webcamName;
    OpenCvCamera camera;
    FieldConstants.ShippingElementPosition elementPosition;
    int slidePosition;
    boolean depotParking;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap); // Initializes roadrunner
        actuation = new Actuation(drive, this, null, hardwareMap); // Initializes the actuation class
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // sets the ID for the camera
        pipeline = new Pipeline(); // Initializes the pipeline for the camera
        webcamName = hardwareMap.get(WebcamName.class, "webcam1"); // Gets the webcam from the hardware map
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId); // Initializes the camera
        camera.setPipeline(pipeline); // Sets the cameras pipeline
        drive.setPoseEstimate(FieldConstants.blueCarouselStart); // Sets the starting position for the robot
        depotParking = false;
        Trajectory toCarousel = drive.trajectoryBuilder(FieldConstants.blueCarouselStart) // Build the trajectory to move from the starting position to the carousel
                .lineToLinearHeading(FieldConstants.blueCarousel)
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
        sleep(5000);
        drive.followTrajectory(toCarousel); // move to carousel

        actuation.carouselSpinBlue(); // Spin the carousel spinner for the blue side
        sleep(3000);
        actuation.stopCarousel(); // Stop the carousel
        sleep(200); //armaan is sus

        Trajectory transition = drive.trajectoryBuilder(drive.getPoseEstimate()) // Builds the default case for the transition point trajectory
                .lineToLinearHeading(new Pose2d(FieldConstants.transitionPointBlue.getX(), FieldConstants.transitionPointBlue.getY(), Math.toRadians(180)))
                .build();
        Trajectory toHub = drive.trajectoryBuilder(transition.end()) // Builds the trajectory from the transition point to the shipping hub
                .lineToConstantHeading(new Vector2d(FieldConstants.blueShippingHub.getX() - 19.5, FieldConstants.blueShippingHub.getY() + 1))
                .build();
        switch (elementPosition) { // change transition point based on the position of the capstone (from cv) (so it doesnt displace it)
            case CENTER:
                break;
            case LEFT:
                toHub = drive.trajectoryBuilder(transition.end())
                        .lineToConstantHeading(new Vector2d(FieldConstants.blueShippingHub.getX() - 22, FieldConstants.blueShippingHub.getY()))
                        .build();
                break;
            case RIGHT:
                toHub = drive.trajectoryBuilder(transition.end())
                        .lineToConstantHeading(new Vector2d(FieldConstants.blueShippingHub.getX() - 17, FieldConstants.blueShippingHub.getY() + 2))
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

        Trajectory transition2 = drive.trajectoryBuilder(toHub.end())
                .lineToSplineHeading(new Pose2d(FieldConstants.transitionPointBlue2.getX(), FieldConstants.transitionPointBlue2.getY(), Math.toRadians(0)))
                .build();

        Trajectory toDepot = drive.trajectoryBuilder(toHub.end()) // Build the trajectory to the depot from the shipping hub position
                .lineToSplineHeading(new Pose2d(FieldConstants.blueDepot.getX(), FieldConstants.blueDepot.getY(), Math.toRadians(0)))
                .build();

        Trajectory transition3 = drive.trajectoryBuilder(toDepot.end()) // A transition point from the depot to the warehouse to avoid hitting the capstone
                .strafeTo(FieldConstants.transitionPointBlue3)
                .build();

        Trajectory toPark = drive.trajectoryBuilder(transition3.end())
                .splineToConstantHeading(FieldConstants.evasiveBlue, Math.toRadians(0))// Build the trajectory from the transition point to the parking spot
                .splineToConstantHeading(FieldConstants.blueWarehouse, Math.toRadians(0))
                .build();

        actuation.slideReset(); // Reset the slide position
        drive.followTrajectory(transition2); // Follow the trajectory to the depot
        if (depotParking) {
            drive.followTrajectory(toDepot); // Follow the trajectory to the depot
        }
        else {
            drive.followTrajectory(transition3); // Move to the transition point
            drive.followTrajectory(toPark); // Move to the parking spot and park
        }
    }
}
