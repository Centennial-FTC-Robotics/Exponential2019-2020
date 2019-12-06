package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(group = "Autonomous", name = "LeftCornerAutoPath")
public class LeftCornerAutoPath extends Exponential_Methods {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AQmuIUP/////AAAAGR6dNDzwEU07h7tcmZJ6YVoz5iaF8njoWsXQT5HnCiI/oFwiFmt4HHTLtLcEhHCU5ynokJgYSvbI32dfC2rOvqmw81MMzknAwxKxMitf8moiK62jdqxNGADODm/SUvu5a5XrAnzc7seCtD2/d5bAIv1ZuseHcK+oInFHZTi+3BvhbUyYNvnVb0tQEAv8oimzjiQW18dSUcEcB/d6QNGDvaDHpxuRCJXt8U3ShJfBWWQEex0Vp6rrb011z8KxU+dRMvGjaIy+P2p5GbWXGJn/yJS9oxuwDn3zU6kcQoAwI7mUgAw5zBGxxM+P35DoDqiOja6ST6HzDszHxClBm2dvTRP7C4DEj0gPkhX3LtBgdolt";

    private static final int MIDDLE_SCREEN = 640;
    //private VuforiaLocalizer vuforia;

    //private TFObjectDetector tfod;


    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
        waitForStart();
        initVuforia();
        initTfod();
        telemetry.addData("in run op mode", "");
        telemetry.update();
        findAndGetSkystone();

    }

    public void findAndGetSkystone() {
        //testing github
        telemetry.addData("infindnandgetsttkyyststone", "");
        telemetry.update();
        move(30, 0, .5); //go forward 2 feet
        turnRelative(-50);
        moveBackLeftUntilCenter();
        move(12 * Math.sqrt(2), 0, .25);
        turnRelative(50);
        move(-40, 0, .25);
        move(0, -144, .5);
    }

    public void moveBackLeftUntilCenter() {
        boolean center = false;
        if (tfod != null) {
            tfod.activate();
        }
        if (opModeIsActive()) {

            while (!center) {
//                frontLeft.setPower(0);
//                frontRight.setPower(-.1);
//                backLeft.setPower(-.1);
//                backRight.setPower(0);
                frontLeft.setPower(-.1);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(-.1);
                //setPowerDriveMotors(.05, -.05, 0, -.05);
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        int i = 0;
                        telemetry.addData(String.format("updatedRecognitions size: (%d)", i), updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                float stoneMiddlePosition = (recognition.getTop() + recognition.getBottom()) / 2;
                                center = getInCenter(stoneMiddlePosition);
                                telemetry.addData(String.format("stoneMiddlePosition (%d)", i), stoneMiddlePosition);
                            }
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        }
                        telemetry.update();
                    }
                }
            }
            setPowerDriveMotors(0);
        }
    }


    public boolean getInCenter(float center) {
        return center + 200 > MIDDLE_SCREEN && MIDDLE_SCREEN > center - 200;
    }


}
