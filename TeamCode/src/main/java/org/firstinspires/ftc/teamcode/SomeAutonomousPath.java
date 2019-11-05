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

        waitForStart();
        initVuforia();
        initTfod();


        while(continueMoving()) {

        }
    }
    public boolean continueMoving() {
        boolean returnBoolean = true;
        telemetry.update();
        if (tfod != null) {
            tfod.activate();
        }
        if (opModeIsActive()) {

            while (opModeIsActive()) {
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
                                float stoneMiddlePosition = (recognition.getRight() + recognition.getLeft()) / 2;
                                if (stoneMiddlePosition + 100 > MIDDLE_SCREEN && MIDDLE_SCREEN > stoneMiddlePosition - 100) {
                                    returnBoolean = false;
                                }
                                telemetry.addData(String.format("stoneMiddlePosition (%d)", i), stoneMiddlePosition);
                                telemetry.addData(String.format("returnBoolean (%b", i), returnBoolean); //TODO: TEST THIS
                                telemetry.update();
                            }

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            /*telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());*/
                            //telemetry.addData(String.format("TOP (%d)", i), "%.03f", recognition.getTop());
                            //telemetry.addData(String.format("BOTTOM (%d)", i), "%.03f", recognition.getBottom());
                            //telemetry.addData(String.format("RIGHT (%d)", i), "%.03f", recognition.getRight());
                            //telemetry.addData(String.format("LEFT (%d)", i), "%.03f", recognition.getLeft());
                            //telemetry.addData("Angle ", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            telemetry.addData(String.format("BOTTOM - TOP (%d)", i), "%.03f", recognition.getBottom() - recognition.getTop());
                            telemetry.addData(String.format("RIGHT - LEFT (%d)", i), "%.03f", recognition.getRight() - recognition.getLeft());
                            //telemetry.addData(String.format("getHeight() (%d)", i), recognition.getHeight());
                            //telemetry.addData(String.format("getImageHeight() (%d)", i), recognition.getImageHeight());
                            //telemetry.addData(String.format("getWidth() (%d)", i), recognition.getWidth());
                            //telemetry.addData(String.format("getImageWidth (%d)", i), recognition.getImageWidth());

                        }
                        telemetry.update();

                    }
                }
            }
        }
        return returnBoolean;
    }
}
