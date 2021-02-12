package org.firstinspires.ftc.teamcode;

public class BlueAutoB extends VortechsMethods {
    public void runOpMode() throws InterruptedException {

        waitForStart();
        //moveRelative(0,30); //Moves forward into white line
        turnRelative(30); //Turn towards square B
        //moveRelative(0,20); //Moves into square B
        turnRelative(330); //Turn so back is facing ring Goal
        //moveRelative(0,20); //Head back behind white line
        launch(2,3); //Shoot rings
        //moveRelative(0,-10); //Park onto white line

        /*
        //If the bot is on the right side:
        moveRelative(0,30); //Moves forward into white line
        turnRelative(-30); //Turn towards square B
        moveRelative(0,20); //Moves into square B
        turnRelative(-330); //Turn so back is facing ring Goal
        moveRelative(0,20); //Head back behind white line
        launch(2,3); //Shoot rings
        moveRelative(0,-10); //Park onto white line
         */

    }
}