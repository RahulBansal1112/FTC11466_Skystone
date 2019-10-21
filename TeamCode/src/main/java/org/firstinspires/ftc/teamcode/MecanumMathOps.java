package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumMathOps {
    private final int ENCODER_TICKS_PER_INCH = (int) (288/(2.6 * 4 * Math.PI));
    ;

    //See if we can make all the methods static (probably not)?
    private double speed = 1.0;
    private double accelerationPerMilli = 0;
    private long timeAccelerating = 0;
    private long timeMoving = 0;

    //telemetry
    private Telemetry telemetry;
    //motors

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    //powers
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    private double pFlPower = 0;
    private double pFrPower = 0;
    private double pBlPower = 0;
    private double pBrPower = 0;

    public MecanumMathOps(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, Telemetry telemetry){
        this.leftFrontDrive = leftFront;
        this.rightFrontDrive = rightFront;
        this.leftBackDrive  = leftBack;
        this.rightBackDrive = rightBack;
        this.telemetry = telemetry;

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public double getFrontLeftMotorP(){
        return this.flPower;// * this.speed;
    }//this is an edit

    public double getFrontRightMotorP(){
        return this.frPower;// * this.speed;
    }

    public double getBackLeftMotorP(){
        return this.blPower;// * this.speed;
    }

    public double getBackRightMotorP(){
        return this.brPower;// * this.speed;
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return this.speed;
    }

    public void moveIndefinitely(){//move without a designated stop time update so it's intuitive
        this.moveForTime(10000000);//technically a bad solution, but it works
    }

    public void moveForTime(long milliseconds){
        this.timeMoving = milliseconds;
    }//update so it's intuitive

    public boolean isAccelerating(){
        return this.timeAccelerating > 0;
    }

    public void accelerateLinearly(double deltaSpeed,long milliseconds) {//change in power multiplier (-1,1)
        //Make sure that after the acceleration we don't goc over the speed of 1, we may want to change the way rather than
        //changing the step speed, we change the amount of time the acceleration takes palce
        if (this.speed + deltaSpeed > 1.0) {
            deltaSpeed = 1.0-this.speed;
        } else if (this.speed + deltaSpeed <0){//perhaps we make it so minimum of speed is -1? Investigate this later
            deltaSpeed = this.speed;// check these two later
        }
        this.accelerationPerMilli = deltaSpeed/milliseconds;
        this.timeAccelerating = milliseconds;
        this.timeMoving = Math.max(this.timeMoving,timeAccelerating);

    }

    public void strafeAndTurn(double x,double y,double r){
        this.flPower = x + y + r;
        this.frPower = - x + y - r;
        this.blPower = - x + y + r;
        this.brPower = x + y - r;

        double scalar = maxAbsValue(this.flPower, this.frPower, this.blPower, this.brPower);

        if (scalar > 1) {
            this.flPower /= scalar;
            this.frPower /= scalar;
            this.blPower /= scalar;
            this.brPower /= scalar;
        }
    }

    private double maxAbsValue(double a, double b, double c, double  d){

        double a1 = Math.abs(a);
        double b1 = Math.abs(b);
        double c1 = Math.abs(c);
        double d1 = Math.abs(d);

        return (Math.max(Math.max(a1, b1), Math.max(c1, d1)));

    }

    /*
    public void update(long dt) {//dt is in milliseconds not needed for first test update this method so it's intuitive
        this.timeAccelerating -= dt;
        if(this.timeAccelerating < 0){
            this.timeAccelerating = 0;//technically not needed, but may be useful for telemetry purposes
            this.accelerationPerMilli = 0;//no longer accelerating
        }
        this.speed += this.accelerationPerMilli * dt;

        //if still moving
        this.timeMoving -= dt;
        if (this.timeMoving < 0){
            this.timeMoving = 0;
        }
    }
    */

    public void moveForAWhile(long milli, double x, double y, double r, double maxAccelerationPerMilli){
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();
        //Might be inside while loop
        this.strafeAndTurn(x, y, r);
        while(runTime.milliseconds() < milli){
            this.updatePowersSmoothly(5, maxAccelerationPerMilli);
        }

    }

    public void moveInches(double inches, double x, double y){

        //Let all motors run using encoder to get ticks
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Set target position(going straight) to the motors
        //Not sure if it will work correctly. Check later
        leftFrontDrive.setTargetPosition((int)(leftFrontDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        leftBackDrive.setTargetPosition((int)(leftBackDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        rightFrontDrive.setTargetPosition((int)(rightFrontDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        rightBackDrive.setTargetPosition((int)(rightBackDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));


        while (leftFrontDrive.getCurrentPosition() < leftFrontDrive.getTargetPosition() ||
                rightBackDrive.getCurrentPosition() < rightBackDrive.getTargetPosition() ||
                rightFrontDrive.getCurrentPosition() < rightFrontDrive.getTargetPosition() ||
                leftBackDrive.getCurrentPosition() < leftBackDrive.getTargetPosition()) {
            telemetry.addData("Motors", "left back(%.2f), left front (%.2f)" +
                    "right back(%.2f), right front(%.2f)", this.blPower, this.flPower, this.brPower, this.frPower);
        }
        
        //Reset back to original state
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Make sure to check later
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnDegrees(double r){

    }


    public void updatePowers(){//perhaps implement auto acceleration
        this.rightFrontDrive.setPower(this.getFrontRightMotorP());
        this.rightBackDrive.setPower(this.getBackRightMotorP());
        this.leftBackDrive.setPower(this.getBackLeftMotorP());
        this.leftFrontDrive.setPower(this.getFrontLeftMotorP());

    }

    public void updatePowersSmoothly(long dt,double maxPowerChangePerMilli){//implements auto acceleration
        this.pFlPower = Range.clip(this.pFlPower + Range.clip(this.flPower-pFlPower,-maxPowerChangePerMilli * dt,maxPowerChangePerMilli*dt),-1,1);
        this.pFrPower = Range.clip(this.pFrPower + Range.clip(this.frPower-pFrPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);
        this.pBlPower = Range.clip(this.pBlPower + Range.clip(this.blPower-pBlPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);
        this.pBrPower = Range.clip(this.pBrPower + Range.clip(this.brPower-pBrPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);

        this.rightFrontDrive.setPower(this.pFrPower);
        this.leftFrontDrive.setPower(this.pFlPower);
        this.rightBackDrive.setPower(this.pBrPower);
        this.leftBackDrive.setPower(this.pBlPower);
    }



}
