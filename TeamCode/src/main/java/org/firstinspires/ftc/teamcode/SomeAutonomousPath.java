package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(group = "Autonomous", name = "SomeAutonomousPath")
public class SomeAutonomousPath extends Exponential_Methods {

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

        findAndGetSkystone();

    }

    public void findAndGetSkystone() {
        boolean left = false;
        boolean center = false;
        boolean right = false;
        if (tfod != null) {
            tfod.activate();
        }
        if (opModeIsActive()) {

            while (!left && !center && !right) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.

                    int i = 0;
                    telemetry.addData(String.format("updatedRecognitions size: (%d)", i), updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                            float stoneMiddlePosition = (recognition.getTop() + recognition.getBottom()) / 2;
                            center = getInCenter(stoneMiddlePosition);
                            left = getInLeft(stoneMiddlePosition);
                            right = getInRight(stoneMiddlePosition);

                            telemetry.addData(String.format("stoneMiddlePosition (%d)", i), stoneMiddlePosition);
                            telemetry.addData(String.format("leftBool (%b)", i), left);
                            telemetry.addData(String.format("centerBool (%b)", i), center);
                            telemetry.addData(String.format("rightBool (%b)", i), right);
                        }

                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());


                    }
                    telemetry.update();

                }
            }
            }
        }

        if (left) {//turn left 45 deg, go right until skystone in middle, go forwards
            move(12, 12, .5, -.5); //go diagonally 1 foot
            turnRelative(45);
            moveBackRightUntilCenter();
        }

        if (center) {

        }

        if (right) {

        }
    }

    public void moveBackRightUntilCenter() {
        boolean center = false;
        if (tfod != null) {
            tfod.activate();
        }
        if (opModeIsActive()) {

            while (!center) {
                setPowerDriveMotors(.05, -.05, 0, -.05); //going right
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

    public boolean getInLeft(float center) {
        return center + 50 < MIDDLE_SCREEN;
    }

    public boolean getInCenter(float center) {
        return center + 50 > MIDDLE_SCREEN && MIDDLE_SCREEN > center - 50;
    }

    public boolean getInRight(float center) {
        return center - 50 > MIDDLE_SCREEN;
    }

}
