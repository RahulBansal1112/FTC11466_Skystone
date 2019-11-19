package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumMathOps {
    //todo this is only for test chasis
    private final double ENCODER_TICKS_PER_INCH = (288./(4 * Math.PI));


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

    private DcMotor lift;

    //powers
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private double liftPower = 0;

    private double pFlPower = 0;
    private double pFrPower = 0;
    private double pBlPower = 0;
    private double pBrPower = 0;
    private double pLiftPower = 0;

    private SkystoneMover_LinearOpMode mover;

    public MecanumMathOps(SkystoneMover_LinearOpMode mover, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, Telemetry telemetry){
        this.leftFrontDrive = leftFront;
        this.rightFrontDrive = rightFront;
        this.leftBackDrive  = leftBack;
        this.rightBackDrive = rightBack;
        //this.lift = lift;

        this.telemetry = telemetry;
        this.mover = mover;

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

    }

    public MecanumMathOps(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, Telemetry telemetry){
        this.leftFrontDrive = leftFront;
        this.rightFrontDrive = rightFront;
        this.leftBackDrive  = leftBack;
        this.rightBackDrive = rightBack;
        //this.lift = lift;
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

    public double getLiftP() {  return this.liftPower;  }

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

    public void initLift() {

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveLiftDown(double power) {

        //amount of revs needed to go from lift start to end: ~3

        if (this.lift.getCurrentPosition() >= 288 * 0.25) {
            this.liftPower = -power;
        }
        else {
            this.liftPower = power;
        }

    }

    public void moveLiftUp(double power) {

        if (this.lift.getCurrentPosition() <= 288 * 2.5) {
            this.liftPower = power;
        }
        else {
            this.liftPower = -power;
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

    public double calculateCounts(double inches){

        return 288 * inches/(4 * Math.PI) * 2.6;

    }

    public void moveInches(double inches, double x, double y){
        telemetry.addData("MOVEINCHES","started");

        //Set target position(going straight) to the motors
        //Not sure if it will work correctly. Check later
        //leftFrontDrive.setTargetPosition((int)(leftFrontDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        //leftBackDrive.setTargetPosition((int)(leftBackDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        //rightFrontDrive.setTargetPosition((int)(rightFrontDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));
        //rightBackDrive.setTargetPosition((int)(rightBackDrive.getCurrentPosition() + ENCODER_TICKS_PER_INCH * inches));

        //Let all motors run using encoder to get ticks
        double scalar = 1/Math.hypot(x,y);
        this.flPower = (x + y)*scalar;
        this.frPower = (- x + y)*scalar;
        this.blPower = (- x + y)*scalar;
        this.brPower = (x + y)*scalar;



        int lbStrt = leftBackDrive.getCurrentPosition();
        int lfStrt = leftFrontDrive.getCurrentPosition();
        int rbStrt = rightBackDrive.getCurrentPosition();
        int rfStrt = rightFrontDrive.getCurrentPosition();

        int lbTrgt = leftBackDrive.getCurrentPosition() + (int) (ENCODER_TICKS_PER_INCH * inches * this.blPower);
        int lfTrgt = leftFrontDrive.getCurrentPosition() + (int) (ENCODER_TICKS_PER_INCH * inches * this.flPower);
        int rbTrgt = rightBackDrive.getCurrentPosition() + (int) (ENCODER_TICKS_PER_INCH * inches * this.brPower);
        int rfTrgt = rightFrontDrive.getCurrentPosition() + (int) (ENCODER_TICKS_PER_INCH * inches * this.frPower);

        //int lbTrgt = leftBackDrive.getCurrentPosition() + (int) (calculateCounts(inches) * this.blPower);
        //int lfTrgt = leftFrontDrive.getCurrentPosition() + (int) (calculateCounts(inches) * this.flPower);
        //int rbTrgt = rightBackDrive.getCurrentPosition() + (int) (calculateCounts(inches) * this.brPower);
        //int rfTrgt = rightFrontDrive.getCurrentPosition() + (int) (calculateCounts(inches) * this.frPower);


        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /*
        this.strafeAndTurn(x,y,0);
        this.flPower = Math.abs(this.flPower);
        this.updatePowers();
        */
        ElapsedTime runTime = new ElapsedTime();
        double prevTime = runTime.time();

        //this.updatePowers();
        int delta = 20;
        while((Math.abs(lfTrgt- leftFrontDrive.getCurrentPosition()) +
                Math.abs(rfTrgt - rightFrontDrive.getCurrentPosition())+
                Math.abs(lbTrgt- leftBackDrive.getCurrentPosition()) +
                Math.abs(rbTrgt - rightBackDrive.getCurrentPosition()) > delta*4
                )&& mover.opModeIsActive()) {

            if (Math.abs(lfTrgt- leftFrontDrive.getCurrentPosition()) > delta)
                this.flPower = (float) (lfTrgt-this.leftFrontDrive.getCurrentPosition())/Math.abs(lfTrgt-lfStrt);
            else
                this.flPower = 0;

            if (Math.abs(rfTrgt - rightFrontDrive.getCurrentPosition()) > delta)
                this.frPower = (float) (rfTrgt-this.rightFrontDrive.getCurrentPosition())/Math.abs(rfTrgt-rfStrt);
            else
                this.frPower = 0;

            if (Math.abs(lbTrgt- leftBackDrive.getCurrentPosition()) > delta)
                this.blPower = (float) (lbTrgt-this.leftBackDrive.getCurrentPosition())/Math.abs(lbTrgt-lbStrt);
            else
                this.blPower = 0;

            if (Math.abs(rbTrgt - rightBackDrive.getCurrentPosition()) > delta)
                this.brPower = (float) (rbTrgt-this.rightBackDrive.getCurrentPosition())/Math.abs(rbTrgt-rbStrt);
            else
                this.brPower = 0;


            if (Math.abs(this.flPower) < 0.2f && this.flPower != 0){
                this.flPower = (this.flPower > 0) ? 0.2f:-0.2f;
            }
            if (Math.abs(this.frPower) < 0.2f && this.frPower != 0){
                this.frPower = (this.frPower > 0) ? 0.2f:-0.2f;
            }
            if (Math.abs(this.blPower) < 0.2f && this.blPower != 0){
                this.blPower = (this.blPower > 0) ? 0.2f:-0.2f;
            }
            if (Math.abs(this.brPower) < 0.2f && this.brPower != 0){
                this.brPower = (this.brPower > 0) ? 0.2f:-0.2f;
            }

            if (Math.abs(this.rightBackDrive.getCurrentPosition()-rbTrgt)/(float)Math.abs(rbTrgt-rbStrt) > 0.5 &&
                    Math.abs(this.rightFrontDrive.getCurrentPosition()-rfTrgt)/(float)Math.abs(rfTrgt-rfStrt) > 0.5 &&
                    Math.abs(this.leftBackDrive.getCurrentPosition()-lbTrgt)/(float)Math.abs(lbTrgt-lbStrt) > 0.5 &&
                    Math.abs(this.leftFrontDrive.getCurrentPosition()-lfTrgt)/(float)Math.abs(lfTrgt-lfStrt) > 0.5) {
                this.updatePowersSmoothly((long) (1000L * (runTime.time() - prevTime)), 0.005);
                telemetry.addData("Mode: ", "accelerating");
            }
            else {
//            this.flPower *= 0.5;
//            this.blPower *= 0.5;
//            this.frPower *= 0.5;
//            this.brPower *= 0.5;
            this.updatePowers();
                //telemetry.addData("Mode: ", "not accelerating");
            }
            telemetry.addData("DELTA TIME","dt " + (long)(1000L * (runTime.time()-prevTime)));
            telemetry.addData("Raw Motor Power",
                    "lf(%.2f) rf(%.2f) lb(%.2f) rb(%.2f)",
                    this.flPower, this.frPower, this.blPower,
                    this.brPower);
            telemetry.addData("Motors", "left back(%d), left front (%d)" +
                "right back(%d), right front(%d)", lbTrgt-lbStrt, lfTrgt-lfStrt,rbTrgt-rbStrt,rfTrgt-rfStrt);
            telemetry.addData("Motors", "left back(%d), left front (%d)" +
                    "right back(%d), right front(%d)", lbTrgt-leftBackDrive.getCurrentPosition(), lfTrgt-leftFrontDrive.getCurrentPosition(),rbTrgt-rightBackDrive.getCurrentPosition(),rfTrgt-rightFrontDrive.getCurrentPosition());
            telemetry.update();
            prevTime = runTime.time();

        }
    /*
        while (leftFrontDrive.getCurrentPosition() != leftFrontDrive.getTargetPosition() ||
                rightBackDrive.getCurrentPosition() != rightBackDrive.getTargetPosition() ||
                rightFrontDrive.getCurrentPosition() != rightFrontDrive.getTargetPosition() ||
                leftBackDrive.getCurrentPosition() !=
                        leftBackDrive.getTargetPosition()) {
            //telemetry.addData("motors","left: " + leftBackDrive.getCurrentPosition() + " "
            //telemetry.addData("Motors", "left back(%.2f), left front (%.2f)" +
            //    "right back(%.2f), right front(%.2f)", this.blPower, this.flPower, this.brPower, this.frPower);

        }
        */

        strafeAndTurn(0, 0, 0);
        this.updatePowers();

        telemetry.addData("MOVEINCHES","ended");

        //Reset back to original state
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Make sure to check later
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Raw Motor Power",
                "lf(%.2f) rf(%.2f) lb(%.2f) rb(%.2f)",
                this.flPower, this.frPower, this.blPower,
                this.brPower);
        telemetry.addData("Motors", "left back(%d), left front (%d)" +
                "right back(%d), right front(%d)", lbTrgt-leftBackDrive.getCurrentPosition(), lfTrgt-leftFrontDrive.getCurrentPosition(),rbTrgt-rightBackDrive.getCurrentPosition(),rfTrgt-rightFrontDrive.getCurrentPosition());
        telemetry.update();
    }

    public void turnDegrees(double r){

    }


    public void updatePowers(){//perhaps implement auto acceleration
        this.rightFrontDrive.setPower(this.getFrontRightMotorP());
        this.rightBackDrive.setPower(this.getBackRightMotorP());
        this.leftBackDrive.setPower(this.getBackLeftMotorP());
        this.leftFrontDrive.setPower(this.getFrontLeftMotorP());
        //this.lift.setPower(this.getLiftP());

        this.pFlPower = flPower;
        this.pBlPower = blPower;
        this.pFrPower = frPower;
        this.pBrPower = brPower;
        //this.pLiftPower = liftPower;

    }

    public void updatePowersSmoothly(long dt,double maxPowerChangePerMilli){//implements auto acceleration
        this.pFlPower = Range.clip(this.pFlPower + Range.clip(this.flPower-pFlPower,-maxPowerChangePerMilli * dt,maxPowerChangePerMilli*dt),-1,1);
        this.pFrPower = Range.clip(this.pFrPower + Range.clip(this.frPower-pFrPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);
        this.pBlPower = Range.clip(this.pBlPower + Range.clip(this.blPower-pBlPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);
        this.pBrPower = Range.clip(this.pBrPower + Range.clip(this.brPower-pBrPower,-maxPowerChangePerMilli*dt,maxPowerChangePerMilli*dt),-1,1);
        //this.pLiftPower = Range.clip(this.pLiftPower + Range.clip(this.liftPower - this.pLiftPower, -maxPowerChangePerMilli * dt, maxPowerChangePerMilli * dt), -1, 1);

        this.rightFrontDrive.setPower(this.pFrPower);
        this.leftFrontDrive.setPower(this.pFlPower);
        this.rightBackDrive.setPower(this.pBrPower);
        this.leftBackDrive.setPower(this.pBlPower);
        //this.lift.setPower(this.pLiftPower);

    }



}
