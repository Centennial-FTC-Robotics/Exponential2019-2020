package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "OneTileForwardAuto")
public class OneTileForwardAuto extends Exponential_Methods { // lol ftc is a fucking meme
    public void runOpMode() throws InterruptedException { // starts right on tape, facing forward
        super.runOpMode();
        waitForStart();
        // middle of second tile, on tape
        move(0, TILE_LENGTH + MIDDLE_OF_TILE, .5);
    }
}
