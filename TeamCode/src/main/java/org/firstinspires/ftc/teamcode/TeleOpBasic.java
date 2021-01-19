package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Basic", group = "Tests")
public class TeleOpBasic extends VortechsHardware{

    double MotorPower = 0.0;
    double Steering = 0.0;
    double RightPower = 0.0;
    double LeftPower = 0.0;

    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();

        while(opModeIsActive())
        {
            MotorPower = -gamepad1.left_stick_y;
            Steering = gamepad1.left_stick_x;
            RightPower = MotorPower - Steering;
            LeftPower = MotorPower + Steering;

            frontLeft.setPower(LeftPower);
            backLeft.setPower(LeftPower);
            frontRight.setPower(RightPower);
            backRight.setPower((RightPower));

            if (this.gamepad1.right_trigger > 0) {    //slowmode
                RightPower = RightPower/2.0;
                LeftPower = LeftPower/2.0;
            }

            idle();
        }
    }
}
