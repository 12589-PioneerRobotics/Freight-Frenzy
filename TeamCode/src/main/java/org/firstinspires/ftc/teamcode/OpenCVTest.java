package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvViewport;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class OpenCVTest extends LinearOpMode {
    int cameraViewId;
    WebcamName webcamName;
    OpenCvCamera testCam;
    TestPipeline pipeline;

    @Override
    public void runOpMode() {
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pipeline = new TestPipeline();
        webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        testCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId);
        testCam.setPipeline(pipeline);

        testCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //testCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                testCam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Element Position: ", pipeline.getDetectionResults());
            telemetry.addData("Average 1: ", pipeline.averageS1);
            telemetry.addData("Average 2: ", pipeline.averageS2);

            telemetry.update();

            sleep(100);
        }
    }






}

class TestPipeline extends OpenCvPipeline{


    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point topAnchor1 = new Point(40, 240);
    static final Point topAnchor2 = new Point(290, 240);
    static final Point bottomAnchor1 = new Point(topAnchor1.x + 40, topAnchor1.y + 40);
    static final Point bottomAnchor2 = new Point(topAnchor2.x + 40, topAnchor2.y + 40);

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

        Imgproc.rectangle(input, topAnchor1, bottomAnchor1, GREEN, 1);
        Imgproc.rectangle(input, topAnchor2, bottomAnchor2, BLUE, 1);

        if(averageS1 > 100 && averageS2 > 100){
            if(averageH1 > 20 && averageH1 < 24)
                position = FieldConstants.ShippingElementPosition.LEFT;
            else if(averageH2 > 20 && averageH2 < 24)
                position = FieldConstants.ShippingElementPosition.CENTER;
            else
                position = FieldConstants.ShippingElementPosition.RIGHT;
        }

        else if(averageS1 > 100)
            position = FieldConstants.ShippingElementPosition.LEFT;
        else if(averageS2 > 100)
            position = FieldConstants.ShippingElementPosition.CENTER;
        else
            position = FieldConstants.ShippingElementPosition.RIGHT;

        return input;

    }

    public FieldConstants.ShippingElementPosition getDetectionResults(){
        return position;
    }

}
