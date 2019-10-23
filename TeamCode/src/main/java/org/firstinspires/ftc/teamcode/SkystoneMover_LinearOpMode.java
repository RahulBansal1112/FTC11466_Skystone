package org.firstinspires.ftc.teamcode;

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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="SkystoneMover_LinearOpMode", group="Linear Opmode")
public class SkystoneMover_LinearOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override


    public void runOpMode() {
        //Make instance of MathOps class

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"right_back_drive");

        MecanumMathOps mathOps = new MecanumMathOps(this, leftFrontDrive,leftBackDrive,rightFrontDrive,rightBackDrive,telemetry);

        waitForStart();
        runtime.reset();
        
        //plans for autonomous:
        //get to a stone
        //pick it up
        //move both robot and stone to building zone
        //go back
        //repeat
        //specifics later

        mathOps.moveInches(3, 1, 0);
        telemetry.update();

        //GO STRAIGHT RIGHT
        //sleep(3000);//there's a problem with sleep?
        //mathOps.strafeAndTurn(1,0,0);//STRAIGHT RIGHT
        //updateMotorSpeeds(mathOps);

        //GO STRAIGHT LEFT
        /*sleep(3000);//there's a problem with sleep?
        mathOps.strafeAndTurn(-1,0,0);//STRAIGHT LEFT
        updateMotorSpeeds(mathOps);

        //STRAFE UP
        sleep(3000);//there's a problem with sleep?
        mathOps.strafeAndTurn(0,1,0);//STRAIGHT UP
        updateMotorSpeeds(mathOps);

        //STRAFE DOWN
        sleep(3000);//there's a problem with sleep?
        mathOps.strafeAndTurn(0,-1,0);//STRAIGHT DOWN
        updateMotorSpeeds(mathOps);*/



    }


    private void updateMotorSpeeds(MecanumMathOps mathOps){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower = mathOps.getFrontLeftMotorP();
        double rightFrontPower = mathOps.getFrontRightMotorP();
        double leftBackPower = mathOps.getBackLeftMotorP();
        double rightBackPower = mathOps.getBackRightMotorP();

        leftBackDrive.setPower(leftBackPower);
        leftFrontDrive.setPower(leftFrontPower);
        rightBackDrive.setPower(rightBackPower);
        rightFrontDrive.setPower(rightFrontPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left back(%.2f), left front (%.2f)" +
                "right back(%.2f), right front(%.2f)", leftBackPower, leftFrontPower, rightBackPower, rightFrontPower);
        telemetry.update();
    }
}
