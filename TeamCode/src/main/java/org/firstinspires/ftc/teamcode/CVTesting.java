package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="CV Test")
public class CVTesting extends OpMode {
    Pipeline pipeline;
    int cameraViewId;
    int slidePosition;
    WebcamName webcamName;
    OpenCvCamera camera;
    FieldConstants.ShippingElementPosition elementPosition;
    TelemetryPacket packet;
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {
            "capstone"
    };
    private TFObjectDetector tfod;


    @Override
    public void init() {
        cameraViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pipeline = new Pipeline();
        webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraViewId);
        camera.setPipeline(pipeline);

        packet = new TelemetryPacket();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        elementPosition = pipeline.getDetectionResults();

        if (elementPosition == FieldConstants.ShippingElementPosition.LEFT)
            slidePosition = 1;
        else if (elementPosition == FieldConstants.ShippingElementPosition.CENTER)
            slidePosition = 2;
        else if (elementPosition == FieldConstants.ShippingElementPosition.RIGHT)
            slidePosition = 3;



        telemetry.addData("Position: ", slidePosition);
        telemetry.addData("Element Position: ", pipeline.getDetectionResults());
        telemetry.addData("SAverage 1: ", pipeline.averageS1);
        telemetry.addData("SAverage 2: ", pipeline.averageS2);
        telemetry.addData("HAverage 1: ", pipeline.averageH1);
        telemetry.addData("HAverage 2: ", pipeline.averageH2);
        telemetry.update();
    }
}
