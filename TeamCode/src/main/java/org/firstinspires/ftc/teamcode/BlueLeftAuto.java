package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "BlueLeftAuto")
public class BlueLeftAuto extends VortechsMethods {
    // robot starts on left side
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        int rings = detectRings();
        telemetry.addData("target zone", rings);
        /*if (rings == VortechsMethods.TARGET_A) {
            moveToTargetALeft("blue");
        } else if (rings == VortechsMethods.TARGET_B) {
            moveToTargetBLeft("blue");
        } else if (rings == VortechsMethods.TARGET_C) {
            moveToTargetCLeft("blue");
        }*/
    }


}
