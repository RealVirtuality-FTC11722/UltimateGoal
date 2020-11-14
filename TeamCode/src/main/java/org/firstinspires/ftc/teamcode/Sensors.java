package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the building system using servos
 */

public class Sensors {
    //OpenCvInternalCamera phoneCam;
    OpenCvWebcam webcamName;
    EasyOpenCV.SkystoneDeterminationPipeline pipeline;

    public Sensors(){ //constructor
    }

    public void init(HardwareMap myHWMap){
        //int cameraMonitorViewId = myHWMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHWMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcamName = OpenCvCameraFactory.getInstance().createWebcam(myHWMap.get(WebcamName.class, "Webcam 1"));
        int cameraMonitorViewId = myHWMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHWMap.appContext.getPackageName());
        pipeline = new EasyOpenCV.SkystoneDeterminationPipeline();
        webcamName.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcamName.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcamName.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcamName.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

    }



}