package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "PIDTest")
public class PIDTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        waitForStart();
        super.runOpMode();

        /*move(24,0);
        autoControlWobbleArm();*/

        outakeWithEncoders(0.7);

/*        telemetry.addData("Distance traveled:", frontLeft.getPower());
        telemetry.update();*/
    }
}
