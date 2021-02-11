package org.firstinspires.ftc.teamcode;

public class BlueAutoA extends VortechsMethods {
    public void runOpMode() throws InterruptedException{

        waitForStart();
        //moveRelative(0,70); //Move towards square A
        turnRelative(-30); //Turn facing square A
        //moveRelative(0,15); //Move into square A
        turnRelative(-330); //Turn shooter towards ring goal
        //moveRelative(0,80); //Move back behind white line
        launch(2,3); //Shoot rings
        //moveRelative(0,-5); //Back into white line

        /*
        //If bot is on the right side
        moveRelative(0,70); //Move towards square A
        turnRelative(-45); //Turn facing square A
        moveRelative(0,15); //Move into square A
        turnRelative(-315); //Turn shooter towards ring goal
        moveRelative(0,80); //Move back behind white line
        launch(2,3); //Shoot rings
        moveRelative(0,-5); //Back into white line
        */

    }
}

