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
        drive.setPoseEstimate(FieldConstants.redWarehouseStart); // Sets the starting position for the robot
        Trajectory transition = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(FieldConstants.warehouseTransition)
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
        drive.followTrajectory(transition);

        Trajectory toHub = drive.trajectoryBuilder(transition.end())
                .splineToSplineHeading(new Pose2d(FieldConstants.redShippingHub.getX(), FieldConstants.redShippingHub.getY() - 15), Math.toRadians(-90))
                .build();
        drive.followTrajectory(toHub);
    }
}
