package org.firstinspires.ftc.teamcode;

public class AutonomousBlueLeft extends VortechsMethods {
    public void runOpMode() throws InterruptedException {

        waitForStart();
        moveRelative(0,30);
        moveRelative(0,-15);
        turnRelative(-30);
        moveRelative(0,-25);
        turnRelative(30);
        moveRelative(0,20);
        launch(1,2);
    }
}
