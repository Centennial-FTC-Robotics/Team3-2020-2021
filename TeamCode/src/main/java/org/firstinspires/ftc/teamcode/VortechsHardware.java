package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class VortechsHardware extends LinearOpMode {

    //initialize motors and servos here
    public DcMotorEx frontRight = null;
    public DcMotorEx frontLeft = null;
    public DcMotorEx backRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx leftOutTake = null;
    public DcMotorEx rightOutTake = null;
    public DcMotorEx intakeWheel = null;
   // public Servo conveyor;
   // public Servo grabberArm;
    public BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftOutTake = hardwareMap.get(DcMotorEx.class, "leftOutTake");
        rightOutTake = hardwareMap.get(DcMotorEx.class, "rightOutTake");
        intakeWheel = hardwareMap.get(DcMotorEx.class, "intakeWheel");
       // conveyor = hardwareMap.get(Servo.class, "conveyor");
      //  grabberArm = hardwareMap.get(Servo.class, "grabberArm");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        final DcMotorEx[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
        for(DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        leftOutTake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightOutTake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }
}
