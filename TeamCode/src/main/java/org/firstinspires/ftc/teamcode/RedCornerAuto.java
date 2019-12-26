package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "RedCornerAuto")
public class RedCornerAuto extends Exponential_Methods {

    public static final double TILE_LENGTH = 22.75;
    public void runOpMode() throws InterruptedException {
        int factor = 1;
        super.runOpMode();
        waitForStart();
        run("red");
        /*while(opModeIsActive())
            run("red");*/
    }

      public void run(String color) {
          int factor;
          if (color.equals("red"))
              factor = 1;
          else
              factor = -1;

          move(0, factor * -TILE_LENGTH, 0.5); //move to corner
          move(18, 0, 0.5); //move forward towards stones
          int inchesMoved = grabSkystone(color);

          move(-18, 0, 0.5); //move back (can be cut out)
          move(0, factor * (TILE_LENGTH * 5 - inchesMoved), 0.5); //move through alliance bridge
          move(4.75 + TILE_LENGTH, 0, 0.5); //move to foundation

          extendSlidesTo(3,0.5); //placeholder value rn
          releaseStone(); //drop stone out
          extendSlidesTo(0,0.5);

          //moving foundation
          turnAbsolute(180); //turn around
          toggleHook(true); //grab foundation
          move(4.75 + TILE_LENGTH, factor * 14, 0.5); //move to wall
          turnRelative(factor * 90);
          toggleHook(false);

          move(28.875, 0, 0.5); //parks on tape
      }
}