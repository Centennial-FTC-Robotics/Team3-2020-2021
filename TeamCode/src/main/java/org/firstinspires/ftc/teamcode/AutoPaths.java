package org.firstinspires.ftc.teamcode;

public class AutoPaths extends VortechsMethods {

    public void moveToTargetA() throws InterruptedException {
        waitForStart();
        moveRelative(0, 70); //Move towards square A
        turnRelative(-30); //Turn facing square A
        moveRelative(0, 15); //Move into square A
        turnRelative(-330); //Turn shooter towards ring goal
        moveRelative(0, 80); //Move back behind white line
        launch(2, 3); //Shoot rings
        moveRelative(0, -5); //Back into white line
    }

    public void moveToTargetB() throws InterruptedException {
        waitForStart();
        moveRelative(0, 30); //Moves forward into white line
        turnRelative(30); //Turn towards square B
        moveRelative(0, 20); //Moves into square B
        turnRelative(330); //Turn so back is facing ring Goal
        moveRelative(0, 20); //Head back behind white line
        launch(2, 3); //Shoot rings
        moveRelative(0, -10); //Park onto white line
    }

    public void moveToTargetC() throws InterruptedException {
        moveRelative(0, 40); //Move towards square C
        turnRelative(-30); //Turn towards square C
        moveRelative(0, 10); //Move into square C
        turnRelative(-330); //Turn shooter towards ring goal
        moveRelative(0, 10); //Move behind white line
        launch(2, 3); //Shoot the rings
        moveRelative(0, -5); //Park on white line
    }

    //parks on white launch line
    //idk if we need this
    public void backupAuto() throws InterruptedException {
        moveRelative(0,80);
    }


}
