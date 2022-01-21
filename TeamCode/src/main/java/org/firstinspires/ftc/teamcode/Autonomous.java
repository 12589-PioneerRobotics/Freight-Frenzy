package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        drive = new SampleMecanumDrive(hardwareMap);
        actuation = new Actuation(drive, this, null, hardwareMap);
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pipeline = new Pipeline();
        webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId);
        camera.setPipeline(pipeline);
        drive.setPoseEstimate(FieldConstants.redCarouselStart);

        Trajectory toCarousel = drive.trajectoryBuilder(FieldConstants.redCarouselStart)
                .lineToLinearHeading(FieldConstants.redCarousel)
                .build();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //testCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
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
*/
        elementPosition = pipeline.getDetectionResults();

        if (elementPosition == FieldConstants.ShippingElementPosition.LEFT)
            slidePosition = 1;
        else if (elementPosition == FieldConstants.ShippingElementPosition.CENTER)
            slidePosition = 2;
        else if (elementPosition == FieldConstants.ShippingElementPosition.RIGHT)
            slidePosition = 3;

        telemetry.addLine("Waiting for start");
        telemetry.addData("Element Position: ", elementPosition);
        telemetry.addData("SAverage 1: ", pipeline.averageS1);
        telemetry.addData("SAverage 2: ", pipeline.averageS2);
        telemetry.addData("HAverage 1: ", pipeline.averageH1);
        telemetry.addData("HAverage 2: ", pipeline.averageH2);
        telemetry.update();

        waitForStart();

        Trajectory toHub = drive.trajectoryBuilder(toCarousel.end())
                .lineToLinearHeading(new Pose2d(FieldConstants.redShippingHub.getX() - 4, FieldConstants.redShippingHub.getY() + 4
                , Math.toRadians(135)))
                .build();


        camera.closeCameraDevice();

        drive.followTrajectory(toCarousel);

        actuation.carouselSpinRed();
        sleep(2000);
        actuation.stopCarousel();
        sleep(200);

        drive.followTrajectory(toHub);
        actuation.slideAction(slidePosition);
        sleep(500);
        actuation.depositorOpen();
        sleep(1500);
        actuation.depositorClose();
        sleep(200);

        Trajectory toPark = drive.trajectoryBuilder(toHub.end())
                .lineToSplineHeading(new Pose2d(FieldConstants.redDepot.getX(), FieldConstants.redDepot.getY(), Math.toRadians(-90)))
                .build();

        actuation.slideReset();
        drive.followTrajectory(toPark);
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

        averageH1 = Core.mean(region1).val[0];
        averageH2 = Core.mean(region2).val[0];
        averageS1 = Core.mean(region1).val[2];
        averageS2 = Core.mean(region2).val[2];



        Imgproc.rectangle(input, topAnchor1, bottomAnchor1, BLUE, 1);
        Imgproc.rectangle(input, topAnchor2, bottomAnchor2, BLUE, 1);

        if(averageS1 > 65 && averageS2 > 65){
            if(averageH1 > 60 && averageH1 < 75)
                position = FieldConstants.ShippingElementPosition.LEFT;
            else if(averageH2 > 60 && averageH2 < 75) {
                position = FieldConstants.ShippingElementPosition.CENTER;
            }
            else
                position = FieldConstants.ShippingElementPosition.RIGHT;
        }

        else if(averageS1 > 65)
            position = FieldConstants.ShippingElementPosition.LEFT;
        else if(averageS2 > 65) {
            position = FieldConstants.ShippingElementPosition.CENTER;
        }
        else
            position = FieldConstants.ShippingElementPosition.RIGHT;



        return input;

    }

    public FieldConstants.ShippingElementPosition getDetectionResults() { return position; }

}
