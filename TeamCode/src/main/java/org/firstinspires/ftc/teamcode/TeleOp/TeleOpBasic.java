package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@TeleOp(name = "Teleop Basic", group = "Tests")
public class TeleOpBasic extends VortechsMethods {

    double y = 0.0;   //throttle forward/back
    double x = 0.0;   //throttle left/right
    double xr = 0.0;
    double yr = 0.0;
    double RightPower = 0.0;
    double LeftPower = 0.0;
    double OtherPower = 1.0;
    boolean intakeOn = false;
    int counter;
    double speed = 1.0;
    boolean outTakeOn = false;
    boolean yPressed = false;
    boolean aPressedLast = false;

    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        waitForStart();
        grabberArm.setPosition(1);
        grabberHand.setPosition(0);

        while(opModeIsActive())
        {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            xr = gamepad1.right_stick_x;
            yr = gamepad1.right_stick_y;
            RightPower = y - x;
            LeftPower = y + x;

            if(gamepad1.a) {
                speed = 0.25;        //slowmode
            } else {
                speed = 1.0;
            }

            if (gamepad1.left_trigger>0){
                frontLeft.setPower(-gamepad1.left_trigger/2);
                backLeft.setPower(-gamepad1.left_trigger/2);
                frontRight.setPower(gamepad1.left_trigger/2);
                backRight.setPower(gamepad1.left_trigger/2);
            }
            if (gamepad1.right_trigger>0){
                frontLeft.setPower(gamepad1.right_trigger/2);
                backLeft.setPower(gamepad1.right_trigger/2);
                frontRight.setPower(-gamepad1.right_trigger/2);
                backRight.setPower(-gamepad1.right_trigger/2);
            }

            LeftPower = Range.clip(LeftPower, -1,0.8);
            RightPower = Range.clip(RightPower, -1,0.8);

            frontLeft.setPower(LeftPower * speed);
            frontRight.setPower(RightPower * speed);
            backLeft.setPower(LeftPower * speed);
            backRight.setPower(RightPower * speed);

            if (gamepad1.right_bumper){         //right strafe controlled by right bumper
            frontLeft.setPower(1 * speed);
            frontRight.setPower(-1 * speed);
            backLeft.setPower(-1 * speed);
            backRight.setPower(1 * speed);
            }
            if (gamepad1.left_bumper){         //left strafe controlled by left bumper
            frontLeft.setPower(-1 * speed);
            frontRight.setPower(1 * speed);
            backLeft.setPower(1 * speed);
            backRight.setPower(-1 * speed);
            }

            if (gamepad2.left_bumper){
            grabberArm.setPosition(1);
            grabberHand.setPosition(0);
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

         /* if (gamepad2.y){                       //toggle system
            Toggle();
            telemetry.addData("Boolean", toggle);
            telemetry.update();
            if (toggle){
                if (leftOutTake.getPower() < 0.05){
                    leftOutTake.setPower(-0.64);
                    //rightOutTake.setPower(-0.7);
                }
            }
            else if (toggle == false){
                leftOutTake.setPower(0);
                //rightOutTake.setPower(0);
            }
            }
            */

            if (gamepad2.y){
                leftOutTake.setPower(-0.75);
            }
            else {
                leftOutTake.setPower(0);
            }

            if (gamepad2.b || gamepad1.b){
                intakeWheel.setPower(0.75);
            }
            else {
                intakeWheel.setPower(0);
            }

            //OtherPower = Range.clip(OtherPower, -0.75,0.75);

            if (gamepad2.left_bumper){
            OtherPower = 0.5;
            }
            else{ OtherPower = 1.0;
            }

            conveyor.setPower(gamepad2.left_stick_y * 0.75* OtherPower);

            if (gamepad2.dpad_down) {

                    grabberArm.setPosition(0.3);
                    grabberHand.setPosition(0.7);
                    telemetry.addData("toggle2",toggle2);
                    telemetry.addData("Armpos",grabberArm.getPosition());
                    telemetry.addData("Handpos",grabberHand.getPosition());
                    telemetry.update();
                }
                else if (gamepad2.dpad_up){
                    grabberHand.setPosition(0.25);
                    Timer(500);
                    grabberArm.setPosition(0.9);
                    telemetry.addData("toggle2",toggle2);
                    telemetry.addData("Armpos",grabberArm.getPosition());
                    telemetry.addData("Handpos",grabberHand.getPosition());
                    telemetry.update();
                }

            /*if (gamepad2.b){
                doEverything(.7,0.75,1);
            }
            */
            if (gamepad2.dpad_left){
                grabberHand.setPosition(0.7);
            }
            else if (gamepad2.dpad_right){
                grabberHand.setPosition(0.25);
            }
            if (gamepad2.x){
                intakeWheel.setPower(-0.75);
            }
            else{
                intakeWheel.setPower(0);
            }


           // if (gamepad2.a){ controlWobbleArm();}
            idle();
            if(gamepad2.left_bumper){
                grabberArm.setPosition(1);
                grabberHand.setPosition(0);
            }
            if(gamepad2.right_stick_button){
                doEverything(0.73, 0.7, 0.3);
            }
        }
    }
}
