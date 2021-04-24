/*
This is a test of the autonomous system that moves the robot forward exactly 10 inches.
 */
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous
public class AutonomousTest extends VortechsMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        move(24,0);
        turnRelative(90);
        idle();
    }
}
