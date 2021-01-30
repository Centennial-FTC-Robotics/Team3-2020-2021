package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

@Autonomous
public class detectRingsTest extends VortechsMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(timer.seconds() < 5) {
            telemetry.addData("rings:", super.detectRings());
            telemetry.update();
        }
    }
}
