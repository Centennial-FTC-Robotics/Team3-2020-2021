package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Basic", group = "Tests")
public class TeleOpBasic extends VortechsHardware{

    double MotorPower = 0.0;
    double Steering = 0.0;
    double Turn = 0.0;
    double TurnThrottle = 0.0;
    double RightPower = 0.0;
    double LeftPower = 0.0;
    double OtherPower = 1.0;

    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();

        while(opModeIsActive())
        {
            MotorPower = -gamepad1.left_stick_y;
            Steering = gamepad1.left_stick_x;
            Turn = gamepad1.right_stick_x;
            TurnThrottle = gamepad1.right_stick_y;
            RightPower = MotorPower - Steering;
            LeftPower = MotorPower + Steering;

            frontLeft.setPower(LeftPower);
            backLeft.setPower(RightPower);
            frontRight.setPower(RightPower);
            backRight.setPower((LeftPower));

            frontLeft.setPower(TurnThrottle+Turn);
            backLeft.setPower(TurnThrottle+Turn);
            frontRight.setPower(TurnThrottle-Turn);
            backRight.setPower(TurnThrottle-Turn);

            if (this.gamepad1.right_trigger > 0) {    //slowmode
                RightPower = RightPower/2.0;
                LeftPower = LeftPower/2.0;
            }

            if (this.gamepad2.a){
            intakeWheel.setPower(2*OtherPower);
            }
            else{
            intakeWheel.setPower(0);
            }

            conveyor.setPower(this.gamepad2.left_stick_y);
            grabberArm.setPower(this.gamepad2.left_stick_x);

            if (this.gamepad2.left_trigger>0){
            leftOutTake.setPower(OtherPower);
            rightOutTake.setPower(-OtherPower);
            }
            else {
            leftOutTake.setPower(0.0);
            rightOutTake.setPower(0.0);
            }
            if (this.gamepad2.right_trigger>0){
            OtherPower = OtherPower/2.0;
            }
            idle();
        }
    }
}
