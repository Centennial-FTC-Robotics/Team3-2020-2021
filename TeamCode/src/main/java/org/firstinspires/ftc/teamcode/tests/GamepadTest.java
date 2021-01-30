package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@TeleOp
public class GamepadTest extends VortechsMethods {

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.addData("1",1);
            telemetry.update();
            super.runOpMode();
            waitForStart();
            while(opModeIsActive()) {
                telemetry.addData("Y:",gamepad1.left_stick_y);
                telemetry.addData("X:",gamepad1.left_stick_x);
                telemetry.addData("Left Trigger:",gamepad1.left_trigger);
                telemetry.addData("Right Trigger:",gamepad1.right_trigger);
                telemetry.update();
            }
        }
    }
