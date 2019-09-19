package org.firstinspires.ftc.teamcode;

public class MecanumMathOps {
    //
    private double speed = 0;
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

    public void accelerate(double deltaSpeed,long milliseconds) {
        //to be implemented
    }

    public void strafeAndTurn(double x,double y,double r){
        this.flPower = x + y + r;
        this.frPower = x - y - r;
        this.blPower = x - y + r;
        this.brPower = x + y + r;
    }
    public void update(long dt) {

    }



}
