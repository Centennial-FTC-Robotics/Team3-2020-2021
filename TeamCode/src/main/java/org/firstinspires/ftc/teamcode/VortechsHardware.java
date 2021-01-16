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
    protected DcMotorEx frontRight;
    protected DcMotorEx frontLeft;
    protected DcMotorEx backRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx leftOutTake;
    protected DcMotorEx rightOutTake;
    protected DcMotorEx intakeWheel;
    protected BNO055IMU IMU;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftOutTake = hardwareMap.get(DcMotorEx.class, "leftOutTake");
        rightOutTake = hardwareMap.get(DcMotorEx.class, "rightOutTake");
        intakeWheel = hardwareMap.get(DcMotorEx.class, "intakeWheel");
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);


    }
}
