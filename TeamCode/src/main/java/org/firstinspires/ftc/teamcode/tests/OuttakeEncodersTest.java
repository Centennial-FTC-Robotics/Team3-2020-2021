package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "OuttakeEncodersTest")
public class OuttakeEncodersTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetOutTake();
        waitForStart();
        while(opModeIsActive()) {

            doEverything(0.73, 0.4, 5); //launch rings

            telemetry.addData("outTakePosition:", leftOutTake.getCurrentPosition());
            telemetry.update();

            //target position is around 3500 i think??
        }
    }
}
