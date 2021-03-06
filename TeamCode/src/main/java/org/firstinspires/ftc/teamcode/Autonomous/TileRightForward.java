package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;

@Autonomous(group = "Autonomous", name = "Tile Right Forward")
public class TileRightForward extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        tileSidewaysForwards("right");
    }
}
