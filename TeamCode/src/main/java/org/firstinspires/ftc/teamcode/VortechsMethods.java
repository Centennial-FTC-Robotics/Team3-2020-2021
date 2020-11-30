package org.firstinspires.ftc.teamcode;

class VortechsMethods extends VortechsHardware{

    protected static final double TICKS_PER_INCH = (1120.0/(100.0*Math.PI))*25.4;

    public void launch(double power, long seconds) throws InterruptedException{
        leftOutTake.setPower(power);
        rightOutTake.setPower(-power);
        Thread.sleep(seconds*1000);
    }

    public void intake(long seconds) throws InterruptedException{
        intakeWheel.setPower(1);
        Thread.sleep(seconds*1000);
    }

    

}
