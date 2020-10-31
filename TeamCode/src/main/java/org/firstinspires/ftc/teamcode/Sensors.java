package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the building system using servos
 */

public class Sensors {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ModernRoboticsI2cRangeSensor bridgeSensor;
    ModernRoboticsI2cRangeSensor blockSensor;
    OpenCvInternalCamera phoneCam;
    EasyOpenCV.SkystoneDeterminationPipeline pipeline;

    public Sensors(){ //constructor
    }

    public void init(HardwareMap myHWMap){
        int cameraMonitorViewId = myHWMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myHWMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new EasyOpenCV.SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

    }



}