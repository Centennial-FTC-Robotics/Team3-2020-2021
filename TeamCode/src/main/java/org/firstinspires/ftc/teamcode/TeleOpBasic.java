package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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
    double speed = 1.0;

    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();

        while(opModeIsActive())
        {
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            xr = gamepad1.right_stick_x;
            yr = gamepad1.right_stick_y;
            RightPower = y - x;
            LeftPower = y + x;

            if(gamepad1.right_trigger > 0.05) {
                speed = 0.5;        //slowmode
            } else {
                speed = 1.0;
            }

            LeftPower = Range.clip(LeftPower, -1,0.8);
            RightPower = Range.clip(RightPower, -1,0.8);

            frontLeft.setPower(LeftPower * speed);
            frontRight.setPower(RightPower * speed);
            backLeft.setPower(LeftPower * speed);
            backRight.setPower(RightPower * speed);

            if (gamepad1.right_bumper){         //right strafe controlled by right bumper
            frontLeft.setPower(-1 * speed);
            frontRight.setPower(-1 * speed);
            backLeft.setPower(1 * speed);
            backRight.setPower(1 * speed);
            }
            if (gamepad1.left_bumper){         //left strafe controlled by left bumper
            frontLeft.setPower(-1 * speed);
            frontRight.setPower(1 * speed);
            backLeft.setPower(1 * speed);
            backRight.setPower(-1 * speed);
            }
            
        /*  frontLeft.setPower(LeftPower);         //strafe controlled by left stick
            backLeft.setPower(RightPower);
            frontRight.setPower(RightPower);
            backRight.setPower((LeftPower));  */

        /*    frontLeft.setPower(yr+xr);          //rotation controlled by right stick
            backLeft.setPower(yr+xr);
            frontRight.setPower(yr-xr);
            backRight.setPower(yr-xr);    */

            // toggle intake
          /*  boolean pressed = gamepad2.a;

          //removed a while loop here (for slowmode) since it kept getting stuck

            if(pressed && !toggle){
                intakeWheel.setPower(2);
                telemetry.addData("pressed", pressed);
                telemetry.addData("toggle", toggle);
            }
            toggle = pressed;
   //         intakeWheel.setPower(0);
            telemetry.update();  */

            int num = 0;                    //another intake toggle system
            if (gamepad2.a){
            num = num+1;
                if (num%2 == 1){
                    intakeWheel.setPower(1);
                }
                else if (num%2 == 0){
                    intakeWheel.setPower(0);
                }
            }

            if (gamepad2.x || gamepad1.x){
                intakeWheel.setPower(2);
            }
            else {
                intakeWheel.setPower(0);
            }

            
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

            if (gamepad2.b){
            conveyor.setPower(1);
            leftOutTake.setPower(-OtherPower);
            rightOutTake.setPower(OtherPower);
            }

            if (gamepad2.left_bumper){
            conveyor.setPower(OtherPower);
            }
            else {conveyor.setPower(0.0); }

            grabberArm.setPower(gamepad2.left_stick_x);

            OtherPower = Range.clip(OtherPower, -0.75,0.75);

            if (gamepad2.y){
            leftOutTake.setPower(-OtherPower);
            rightOutTake.setPower(OtherPower);
            }
            else {
            leftOutTake.setPower(0.0);
            rightOutTake.setPower(0.0);
            }
            if (gamepad2.right_bumper){
            OtherPower = 1.0;
            }
            else{ OtherPower = 0.75;
            }
            idle();
        }
    }
}
