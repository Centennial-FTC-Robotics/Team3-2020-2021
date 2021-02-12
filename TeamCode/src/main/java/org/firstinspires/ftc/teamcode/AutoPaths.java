package org.firstinspires.ftc.teamcode;

public class AutoPaths extends VortechsMethods {

    // robot starts on left side

    public void moveToTargetALeft(String color) throws InterruptedException {
        waitForStart();
        move(70, 0); //Move towards square A
        turnRelative(-30); //Turn facing square A
        move(15, 0); //Move into square A
        turnRelative(-330); //Turn shooter towards ring goal
   //     moveRelative(0, 80); //Move back behind white line
        launch(2, 3); //Shoot rings
    //    moveRelative(0, -5); //Back into white line
    }

    public void moveToTargetBLeft(String color) throws InterruptedException {
        waitForStart();
      //  moveRelative(0, 30); //Moves forward into white line
        turnRelative(30); //Turn towards square B
        //moveRelative(0, 20); //Moves into square B
        turnRelative(330); //Turn so back is facing ring Goal
        //moveRelative(0, 20); //Head back behind white line
        launch(2, 3); //Shoot rings
        //moveRelative(0, -10); //Park onto white line
    }

    public void moveToTargetCLeft(String color) throws InterruptedException {
        //moveRelative(0, 40); //Move towards square C
        turnRelative(-30); //Turn towards square C
        //moveRelative(0, 10); //Move into square C
        turnRelative(-330); //Turn shooter towards ring goal
        //moveRelative(0, 10); //Move behind white line
        launch(2, 3); //Shoot the rings
        //moveRelative(0, -5); //Park on white line
    }

    // robot starts on right side

    public void moveToTargetARight(String color) throws InterruptedException {
        waitForStart();
        //moveRelative(0,70); //Move towards square A
        turnRelative(-45); //Turn facing square A
        //moveRelative(0,15); //Move into square A
        turnRelative(-315); //Turn shooter towards ring goal
        //moveRelative(0,80); //Move back behind white line
        launch(2,3); //Shoot rings
        //moveRelative(0,-5); //Back into white line
    }

    public void moveToTargetBRight(String color) throws InterruptedException {
        waitForStart();
        //moveRelative(0,30); //Moves forward into white line
        turnRelative(-30); //Turn towards square B
        //moveRelative(0,20); //Moves into square B
        turnRelative(-330); //Turn so back is facing ring Goal
        //moveRelative(0,20); //Head back behind white line
        launch(2,3); //Shoot rings
        //moveRelative(0,-10); //Park onto white line
    }

    public void moveToTargetCRight(String color) throws InterruptedException {
        //moveRelative(0, 40); //Move towards square C
        //moveRelative(0,40); //Move towards square C
        turnRelative(-45); //Turn towards square C
        //moveRelative(0,17); //Move into square C
        turnRelative(-315); //Turn shooter towards ring goal
        //moveRelative(0,10); //Move behind white line
        launch(2,3); //Shoot the rings
        //moveRelative(0,-5); //Park on white line
    }

    //parks on white launch line

/*    public void backupAuto() throws InterruptedException {
        //moveRelative(0,80);
    }*/


}
