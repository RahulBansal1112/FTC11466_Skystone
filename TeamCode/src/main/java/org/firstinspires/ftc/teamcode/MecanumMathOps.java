package org.firstinspires.ftc.teamcode;

public class MecanumMathOps {
    //
    private double speed = 0;


    public static double getFrontLeftMotorP(double x,double y,double r){
        return x + y + r;
    }

    public static double getFrontRightMotorP(double x,double y,double r){
        return x - y - r;
    }

    public static double getBackLeftMotorP(double x,double y,double r){
        return x - y + r;
    }

    public static double getBackRightMotorP(double x,double y,double r){
        return x - y + r;
    }

    public void accelerate(double deltaSpeed,long millseconds) {

    }

    public void update(long dt) {

    }



}
