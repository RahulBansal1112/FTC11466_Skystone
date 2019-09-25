package org.firstinspires.ftc.teamcode;

public class MecanumMathOps {
    //
    private double speed = 1.0;
    private double accelerationPerMilli = 0;
    private long timeAccelerating = 0;
    //
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;


    public double getFrontLeftMotorP(double x,double y,double r){
        return this.flPower;
    }//this is an edit

    public double getFrontRightMotorP(double x,double y,double r){
        return this.frPower;
    }

    public double getBackLeftMotorP(double x,double y,double r){
        return this.blPower;
    }

    public double getBackRightMotorP(double x,double y,double r){
        return this.brPower;
    }

    public void accelerate(double deltaSpeed,long milliseconds) {//change in power multiplier (-1,1)
        //to be implemented
        if (this.speed + deltaSpeed > 1.0) {
            deltaSpeed = 1.0-this.speed;
        } else if (this.speed + delta)
        this.accelerationPerMilli = deltaSpeed/milliseconds;
        this.timeAccelerating = milliseconds;


    }

    public void strafeAndTurn(double x,double y,double r){
        this.flPower = x + y + r;
        this.frPower = x - y - r;
        this.blPower = x - y + r;
        this.brPower = x + y + r;
    }
    public void update(long dt) {//dt is in milliseconds
        this.speed =

    }



}
