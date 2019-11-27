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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="OneController_Iterative", group="Iterative Opmode")

public class OneController_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor liftMotor;
    private Servo innerPincher;
    private Servo outerPincher;
    private Servo clamper1;
    private Servo clamper2;
    private Servo foundationMech;

    private MecanumMathOps mathOps;

    private boolean driveMode; //true = acceleration, false = none
    private double clampPosition;
    private static final double MAX_POSITION = 1.0;
    private static final double MIN_POSITION = 0.0;
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

        driveMode = false;

        clampPosition = 0.5;


        //lift = hardwareMap.get(DcMotor.class, "lift");
        //mathOps.initLift(); //MAKE SURE THE LIFT IS ALL THE WAY DOWN WHEN STARTING.

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*clamper1 = hardwareMap.get(Servo.class, "clamper1");
        clamper2 = hardwareMap.get(Servo.class, "clamper2");

        innerPincher = hardwareMap.get(Servo.class, "innerPincher");
        outerPincher = hardwareMap.get(Servo.class, "outerPincher");*/

        mathOps = new MecanumMathOps(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive,telemetry);

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

        mathOps.strafeAndTurn(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);
        //if (getLiftDown()) {

            //mathOps.moveLiftDown(0.25);
            //telemetry.addData("Lift direction: ", "down");

        //}
        //else if (getLiftUp()) {

            //mathOps.moveLiftUp(0.25);
            //telemetry.addData("Lift direction: ", "up");

        //}

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
        // The last two if-statements won't work because we have to set the run mode
        if(this.moveLiftUp()){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition() - 1);
        }
        if(this.moveLiftDown()){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + 1);//see what this means
        } todo add this to the config file

        */


        if (gamepad1.x) {
            changeDriveMode();
        }

        if (gamepad1.left_bumper) {

            clampPosition = 0;

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString()+ " " + gamepad1.left_stick_x + " " + gamepad1.left_stick_y);
        telemetry.addData("Motors Turning", "turn(%.2f)", gamepad1.right_stick_x);
        telemetry.addData("Acceleration on:", driveMode);
        telemetry.addData("Raw Motor Power",
                "lf(%.2f) rf(%.2f) lb(%.2f) rb(%.2f)",
                mathOps.getFrontLeftMotorP(), mathOps.getFrontRightMotorP(), mathOps.getBackLeftMotorP(),
                mathOps.getBackRightMotorP());
        telemetry.addData("Servo Position", foundationMech.getPosition());

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

        foundationMech.setPosition(Range.clip(clampPosition, MIN_POSITION, MAX_POSITION));

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
    private boolean getPincherInner() {
        return gamepad1.right_bumper;
    }
    private boolean getLiftUp() {
        return gamepad1.dpad_up;
    }
    private boolean getLiftDown() {
        return gamepad1.dpad_down;
    }
    private void moveLiftUp() {

    }

}


