package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Basic", group = "Tests")
public class TeleOpBasic extends VortechsHardware{

    double y = 0.0;   //throttle forward/back
    double x = 0.0;   //throttle left/right
    double xr = 0.0;
    double yr = 0.0;
    double RightPower = 0.0;
    double LeftPower = 0.0;
    double OtherPower = 1.0;
    boolean intakeOn = false;
    boolean toggle = false;
    int counter;

    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();

        while(opModeIsActive())
        {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            xr = gamepad1.right_stick_x;
            yr = gamepad1.right_stick_y;
            RightPower = y - x;
            LeftPower = y + x;

            frontLeft.setPower(LeftPower);
            frontRight.setPower(RightPower);
            backLeft.setPower(LeftPower);
            backRight.setPower(RightPower);

            if (gamepad1.right_bumper){         //right strafe controlled by right bumper
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(-1);
            backRight.setPower(1);
            }
            if (gamepad1.left_bumper){         //left strafe controlled by left bumper
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backLeft.setPower(1);
            backRight.setPower(-1);
            }

        /*  frontLeft.setPower(LeftPower);         //strafe controlled by left stick
            backLeft.setPower(RightPower);
            frontRight.setPower(RightPower);
            backRight.setPower((LeftPower));  */

        /*    frontLeft.setPower(yr+xr);          //rotation controlled by right stick
            backLeft.setPower(yr+xr);
            frontRight.setPower(yr-xr);
            backRight.setPower(yr-xr);    */

            while (gamepad1.right_trigger > 0) {    //slowmode
                RightPower = RightPower/2.0;
                LeftPower = LeftPower/2.0;
            }

            // toggle intake
            boolean pressed = gamepad2.a;

            if(pressed && !toggle){
                intakeWheel.setPower(2);
                telemetry.addData("pressed", pressed);
                telemetry.addData("toggle", toggle);
            }
            toggle = pressed;
            intakeWheel.setPower(0);
            telemetry.update();

/*            if (gamepad2.a && !intakeOn){
                intakeWheel.setPower(2);
            //intakeOn = !intakeOn;
             //while (intakeOn = true){
             //intakeWheel.setPower(2);
            // }
            }
            else {
            intakeOn = false;
            }*/
            
         /*   boolean intakeOn = false;
            if (gamepad2.a){
                if(intakeOn){
                    intakeOn = false;
                } else {
                    intakeWheel.setPower(2*OtherPower);
                    intakeOn = true;
                }
            } else {
            intakeWheel.setPower(0);
            } //This is so A doesn't have to be held for the outtake to turn on   */

            conveyor.setPower(gamepad2.left_stick_y);
            grabberArm.setPower(gamepad2.left_stick_x);

            if (gamepad2.left_trigger>0){
            leftOutTake.setPower(OtherPower);
            rightOutTake.setPower(-OtherPower);
            }
            else {
            leftOutTake.setPower(0.0);
            rightOutTake.setPower(0.0);
            }
            if (gamepad2.right_bumper){
            OtherPower = 0.5;
            }
            else{ OtherPower = 1.0;
            }
            idle();
        }
    }
}
