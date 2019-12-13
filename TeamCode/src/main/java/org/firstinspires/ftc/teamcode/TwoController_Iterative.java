/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TwoController_Iterative", group="Iterative Opmode")

public class TwoController_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor liftVertical;
    private DcMotor liftAngle;
    private Servo pincher;
    //private Servo outerPincher;
    //private Servo clamper1;
    //private Servo clamper2;
    private Servo foundationMech;

    private MecanumMathOps mathOps;

    private boolean driveMode; //true = acceleration, false = none
    private double clampPosition;
    private double pincherPosition;
    private static final double MAX_POSITION = 1;
    private static final double MIN_POSITION = 0;
    private static final double MIN_PINCHER_POSITION = 0.7;
    private static final double MIN_LIFT_ANGLE = 0;
    private static final double MAX_LIFT_ANGLE = 65;
    //if the left bumper was pressed last frame
    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;
    private boolean aPrevPressed = false;

    private double lift;

    private double liftAngleTarget = 0;
    //initialize clamp + stuff
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        foundationMech = hardwareMap.get(Servo.class, "foundation_mech");
        pincher = hardwareMap.get(Servo.class, "pincher");

        driveMode = false;

        clampPosition = MIN_POSITION;
        pincherPosition = MAX_POSITION;


        liftVertical = hardwareMap.get(DcMotor.class, "lift_vertical");
        liftAngle = hardwareMap.get(DcMotor.class, "lift_angle");
       // mathOps.initLift(); //MAKE SURE THE LIFT IS ALL THE WAY DOWN WHEN STARTING.

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*clamper1 = hardwareMap.get(Servo.class, "clamper1");
        clamper2 = hardwareMap.get(Servo.class, "clamper2");

        innerPincher = hardwareMap.get(Servo.class, "innerPincher");
        outerPincher = hardwareMap.get(Servo.class, "outerPincher");*/

        mathOps = new MecanumMathOps(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive,liftVertical, liftAngle, telemetry);

        mathOps.initLift(); //Assume the lift is all the way down.
        mathOps.initAngle(); //Assume the lift is straight up.
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        //pass

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double lift = getLift(); //getLift gets the joystick y, just setting it to a variable here
        mathOps.strafeAndTurn(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        //need to program it to stop if we get too far

        if (lift < 0) {

            mathOps.moveLiftDown(Math.abs(lift));
            telemetry.addData("Lift direction: ", "down");

        }
        else if (lift > 0) {

            mathOps.moveLiftUp(Math.abs(lift));
            telemetry.addData("Lift direction: ", "up");

        } else {
            mathOps.moveLiftDown(0);
            telemetry.addData("Lift direction: ", "STILL");
        }

        /*
        // If the user presses the right bumper and any of the clamps are outside the chassis, bring
        // back in or else push them back out
        // We might need to put an empty while loop while its moving towards the destination
        if (this.getClamp()){
            if (this.clamper1.getPosition() < 90 && this.clamper2.getPosition() < 90) {//OPen?
                this.clamper1.setPosition(180);//closed?
                this.clamper2.setPosition(180);
            }
            else {//closed?
                this.clamper1.setPosition(0);//open
                this.clamper2.setPosition(0);
            }
        }

        // If the user preses the left bumper and the inner pincher is not half way turned, push it
        // out to grab the stone, otherwise release the stone
        // We might need to put an empty while loop while its moving towards the destination
        if (this.getPincherInner()){
            if (this.innerPincher.getPosition() <45)//Open?
                this.innerPincher.setPosition(90);//CLOSED?
            else//CLOSE?
                this.innerPincher.setPosition(0);//OPEN?
        }
        // If the user presses 'A', the pincher mechanism expands outside the chassis
        // If the user presses 'B', the pincher mechanism will come back to the chassis
        // We might need to put an empty while loop while its moving towards the destination
        if (this.getPincherOuterOpen()){
            outerPincher.setPosition(0);//OPEN?
        }
        if (this.getPincherOuterClose()){
            outerPincher.setPosition(90);//CLOSED?
        }
        /*

        */


        if (gamepad1.x) {
            changeDriveMode(); //drivemode is for acceleration. probably not going to use this
        }

        if (gamepad2.left_bumper && ! this.leftBumperPressed) {

            if (Math.abs(foundationMech.getPosition() - MIN_POSITION) < Math.abs(foundationMech.getPosition() - MAX_POSITION)) {
                clampPosition = MAX_POSITION;
                telemetry.addData("Foundation:",true + " MAX" );

            }
            else {
                clampPosition = MIN_POSITION;
                telemetry.addData("Foundation:",true + " MINIMUM" );

            }
        }else {
            telemetry.addData("Foundation:",false);
        }
        leftBumperPressed = gamepad2.left_bumper; //this loops back around to check if it's been pressed previously


        if (gamepad2.left_stick_button && ! this.aPrevPressed) {
            double mintarget = MIN_POSITION * MecanumMathOps.LIFT_TICKS_PER_REVOLUTION / 360;
            double maxtarget = MAX_POSITION * MecanumMathOps.LIFT_TICKS_PER_REVOLUTION / 360;

            if (Math.abs(maxtarget - liftAngle.getCurrentPosition()) > Math.abs(mintarget-liftAngle.getCurrentPosition())){
                this.liftAngleTarget = MAX_LIFT_ANGLE;
            } else {
                this.liftAngleTarget = MIN_LIFT_ANGLE;
            }
            telemetry.addData("A PRESSED", " A PRESSED");

        }

        //liftAngle.setPower(gamepad2.left_stick_x * 0.15);

        /*
        if (gamepad2.b && this.bPrevPressed) {

            liftAngle.setPower(0);

        }

        bPrevPressed = gamepad2.b;
*/
       mathOps.moveLiftAngle(this.liftAngleTarget);
       this.aPrevPressed = gamepad2.left_stick_button;

        if (gamepad2.right_bumper && ! this.rightBumperPressed) {

            if (Math.abs(pincher.getPosition() - MIN_PINCHER_POSITION) < Math.abs(pincher.getPosition() - MAX_POSITION)) {
                pincherPosition = MAX_POSITION;
                telemetry.addData("Pincher:",true + " MAX" );

            }
            else {
                pincherPosition = MIN_PINCHER_POSITION;
                telemetry.addData("Pincher:",true + " MINIMUM" );

            }
        }else {
            telemetry.addData("Pincher:",false);
        }

        rightBumperPressed = gamepad2.right_bumper;



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString()+ " " + gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.addData("Motors Turning", "turn(%.2f)", gamepad1.right_stick_x);
        telemetry.addData("Acceleration on:", driveMode);
        telemetry.addData("Raw Motor Power",
                "lf(%.2f) rf(%.2f) lb(%.2f) rb(%.2f)",
                mathOps.getFrontLeftMotorP(), mathOps.getFrontRightMotorP(), mathOps.getBackLeftMotorP(),
                mathOps.getBackRightMotorP());
        telemetry.addData("Servo Position", foundationMech.getPosition());
        telemetry.addData("Lift Position", -liftVertical.getCurrentPosition());
        telemetry.addData("Lift Angle Motor","Position " + liftAngle.getCurrentPosition());

        if (driveMode == true) {
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
                mathOps.updatePowersSmoothly(16, 0.01);
            }
            else {
                mathOps.updatePowers();
            }
        }
        else {
            mathOps.updatePowers();
        }

        foundationMech.setPosition(clampPosition);
        pincher.setPosition(pincherPosition);

        //mathOps.updatePowersSmoothly(16,0.001);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void changeDriveMode() {

        if (driveMode == true) driveMode = false;
        else driveMode = true;

    }

    private boolean getPincherOuterOpen() {
        return gamepad1.a;
    }
    private boolean getPincherOuterClose() {
            return gamepad1.b; //placeholder
    }
    private double getLift() {
        return -gamepad2.right_stick_y;
    }
    private boolean getLiftAngle() {
        return gamepad1.right_bumper;
    }


}


