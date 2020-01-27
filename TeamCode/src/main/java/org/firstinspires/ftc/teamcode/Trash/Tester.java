package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

import java.util.List;

@Autonomous(group = "Autonomous", name = "Side camera vuforia stuff")
public class Tester extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    public int findStone(){
        int MIDDLE_SCREEN = 640;
        int CENTER_LEFT = MIDDLE_SCREEN - 200; //set later
        int CENTER_RIGHT = MIDDLE_SCREEN + 200; //set later
        int stonePos = 0;

        if(tfod!= null){
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                        float stoneCamPos = (recognition.getTop() + recognition.getBottom()) / 2;
                        if (stoneCamPos < CENTER_LEFT) {
                            stonePos = 1;
                        } else if (stoneCamPos > CENTER_RIGHT) {
                            stonePos = 3;
                        } else {
                            stonePos = 2;
                        }
                    }
                }
            }
        }
        if(stonePos == 0)
            findStone();
        telemetry.addData("stonePos", stonePos);
        return stonePos;
    }
}