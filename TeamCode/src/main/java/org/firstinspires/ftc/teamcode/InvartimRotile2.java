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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Invartim Rotile2", group="Linear Opmode")
//@Disabled
public class InvartimRotile2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor brat, gheara = null;
    private double motorVal;
    private double servoVal = 0.05;
    private int runtime2, targetPos = 0;
    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        brat  = hardwareMap.get(DcMotor.class, "Brat");
        gheara = hardwareMap.get(DcMotor.class, "Gheara");
        leftServo = hardwareMap.get(Servo.class, "stanga");
        rightServo = hardwareMap.get(Servo.class, "dreapta");


        brat.setDirection(DcMotor.Direction.FORWARD);
        gheara.setDirection(DcMotor.Direction.FORWARD);

        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gheara.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gheara.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start --------------------------------------- (driver presses PLAY)
        waitForStart(); //
        runtime.reset();

        // run until the end of the match ----------------------------------- (driver presses STOP)
        while (opModeIsActive()) {

            if(-gamepad2.left_stick_y > 0.1 || -gamepad2.left_stick_y < -0.1)
                brat.setPower(-gamepad2.left_stick_y);
            else brat.setPower(0);

            if(-gamepad2.right_stick_y > 0.1 || -gamepad2.right_stick_y < -0.1)
                gheara.setPower(-gamepad2.right_stick_y / 2 );
            else gheara.setPower(0);



            if(gamepad2.dpad_up) {
                leftServo.setPosition(leftServo.getPosition() + servoVal);
                rightServo.setPosition(rightServo.getPosition() - servoVal);
                sleep(50);
            }
            else if(gamepad2.dpad_down) {
                leftServo.setPosition(leftServo.getPosition() - servoVal);
                rightServo.setPosition(rightServo.getPosition() + servoVal);
                sleep(50);
            }


//
//            if(gamepad2.dpad_up) {
//                targetPos = brat.getCurrentPosition() - 500;
//                brat.setTargetPosition(-targetPos);
//                brat.setPower(0.5);
//                while (brat.isBusy() && opModeIsActive()) {
//                    if(Math.abs(targetPos - brat.getCurrentPosition()) < 50 )
//                        brat.setPower(0.2);
//                    showData();
//                }
//                brat.setPower(0);
//            }
//            else if(gamepad2.dpad_down) {
//                targetPos = brat.getCurrentPosition() + 500;
//                brat.setTargetPosition(targetPos);
//                brat.setPower(0.5);
//                while (brat.isBusy() && opModeIsActive()) {
//                    if(Math.abs(targetPos - brat.getCurrentPosition()) < 50 )
//                        brat.setPower(0.2);
//                    showData();
//                }
//                brat.setPower(0);
//            }

            showData();

        }
    }

    private void showData() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("CurrentPos: ", brat.getCurrentPosition());
        telemetry.addData("TargetPos: ", targetPos);
        telemetry.addData("BratPower: ", brat.getPower());
        telemetry.addData("GhearaPower", gheara.getPower());
        telemetry.addData("LeftServo Pos: ", leftServo.getPosition());
        telemetry.addData("RightServo Pos: ", rightServo.getPosition());

        telemetry.update();

    }
}
