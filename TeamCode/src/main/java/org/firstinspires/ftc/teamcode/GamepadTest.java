package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class GamepadTest extends VortechsMethods{

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry.addData("1",1);
            telemetry.update();
            super.runOpMode();
            waitForStart();
            while(opModeIsActive()) {
                double y = gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                telemetry.addData("Y:",y);
                telemetry.addData("X:",x);
                telemetry.update();
            }
        }
    }
