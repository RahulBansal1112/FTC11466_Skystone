package org.firstinspires.ftc.teamcode;

public class MecanumMathOps {
    //See if we can make all the methods static?
    private double speed = 1.0;
    private double accelerationPerMilli = 0;
    private long timeAccelerating = 0;
    //
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

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

    public double getSpeed(){
        return this.speed;
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

    }

}
