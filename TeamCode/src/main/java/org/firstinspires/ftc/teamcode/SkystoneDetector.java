package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;



public class SkystoneDetector{
    OpenCvCamera cam;
    OpMode opMode;

    int stonePos = 1;

    public void initialize(OpMode opmode){
        opMode = opmode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        //Open connection to camera
        cam.openCameraDevice();

        cam.setPipeline(new Pipeline());
    }

    public void activate(String color){
        //TODO set later cause I don't know which side is which camera rotation
        if(color.equals("red")){
            cam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
        }
        if(color.equals("blue")){
            cam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
        }
    }


    public void deactivate() {
        cam.stopStreaming();
    }

    class Pipeline extends OpenCvPipeline{
        Mat gray = new Mat();
        Mat croppedGray = new Mat();

        public Mat processFrame(Mat input){
            //TODO set points later
            Rect rectCrop = new Rect(new Point(150,450) , new Point(1090,550));
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            Mat croppedGray = new Mat(gray, rectCrop);

            int stoneSize = 300;  //TODO set later
            int[] divisions = {0, stoneSize*1, stoneSize*2, croppedGray.cols()}; //TODO set later

            double brightnessMin = Double.MAX_VALUE;
            double brightnessAvg = 0;
            int pixels = 0;

            //For each segment
            for(int i=0; i<3; i++){
                //For x
                for(int x = divisions[i]; x < divisions[i+1]; i++){
                    //For y
                    for(int y = 0; y < croppedGray.rows(); y++){
                        brightnessAvg += croppedGray.get(y,x)[0];
                        pixels++;
                    }
                }
                brightnessAvg /= pixels;
                if(brightnessAvg < brightnessMin){
                    brightnessMin = brightnessAvg;
                    stonePos = i;
                }
            }

            return input;
        }



    }

}
