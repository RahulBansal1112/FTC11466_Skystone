package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumMathOps {
    //See if we can make all the methods static (probably not)?
    private double speed = 1.0;
    private double accelerationPerMilli = 0;
    private long timeAccelerating = 0;
    private long timeMoving = 0;

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

    public MecanumMathOps(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack){
        this.leftFrontDrive = leftFront;
        this.rightFrontDrive = rightFront;
        this.leftBackDrive  = leftBack;
        this.rightBackDrive = rightBack;
    }

    public double getFrontLeftMotorP(){
        return this.flPower * this.speed;
    }//this is an edit

    public double getFrontRightMotorP(){
        return this.frPower * this.speed;
    }

    public double getBackLeftMotorP(){
        return this.blPower * this.speed;
    }

    public double getBackRightMotorP(){
        return this.brPower * this.speed;
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public double getSpeed(){
        return this.speed;
    }

    public void moveIndefinitely(){//move without a designated stop time
        this.moveForTime(10000000);//technically a bad solution, but it works

    }

    public void moveForTime(long milliseconds){
        this.timeMoving = milliseconds;
    }

    public boolean isAccelerating(){
        return this.timeAccelerating > 0;
    }

    public void accelerateLinearly(double deltaSpeed,long milliseconds) {//change in power multiplier (-1,1)
        //Make sure that after the acceleration we don't go over the speed of 1, we may want to change the way rather than
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
        this.frPower = x - y - r;
        this.blPower = x - y + r;
        this.brPower = x + y + r;
    }

    public void update(long dt) {//dt is in milliseconds
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
        } else {
            this.rightFrontDrive.setPower(this.getFrontRightMotorP());
            this.rightBackDrive.setPower(this.getFrontRightMotorP());
            this.leftBackDrive.setPower(this.getBackLeftMotorP());
            this.leftFrontDrive.setPower(this.getFrontLeftMotorP());
        }
        }



}
