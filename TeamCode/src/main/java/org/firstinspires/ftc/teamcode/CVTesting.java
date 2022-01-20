package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

        switch (pipeline.getDetectionResults()) {
            case LEFT:
                slidePosition = 1;
            case CENTER:
                slidePosition = 2;
            case RIGHT:
                slidePosition = 3;
        }

        telemetry.addData("Position: ", slidePosition);
        telemetry.update();
    }
}
