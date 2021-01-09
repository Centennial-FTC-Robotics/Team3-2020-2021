package org.firstinspires.ftc.teamcode;

public class BlueAutoC extends VortechsMethods {

    public void runOpMode() throws InterruptedException{

        moveRelative(0,40); //Move towards square C
        turnRelative(-30); //Turn towards square C
        moveRelative(0,10); //Move into square C
        turnRelative(-330); //Turn shooter towards ring goal
        moveRelative(0,10); //Move behind white line
        launch(2,3); //Shoot the rings
        moveRelative(0,-5); //Park on white line

        /*
        //If the bot is on the right side:
        moveRelative(0,40); //Move towards square C
        turnRelative(-45); //Turn towards square C
        moveRelative(0,17; //Move into square C
        turnRelative(-315); //Turn shooter towards ring goal
        moveRelative(0,10); //Move behind white line
        launch(2,3); //Shoot the rings
        moveRelative(0,-5); //Park on white line

        */

    }
}
