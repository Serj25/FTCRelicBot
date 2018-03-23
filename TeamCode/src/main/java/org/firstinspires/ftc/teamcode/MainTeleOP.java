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


@TeleOp(name="MainTeleOP", group="FTC")
//@Disabled
public class MainTeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0, motor1, motor2, motor3 = null;
    private DcMotor slider, arm = null;
    private Servo armGem, clawR, clawL, cJoint;

    private double speedDevider = 1;
    private double sliderInfLimit = 0;
    private double sliderSupLimit = 7500;
    private double armInfLimit = -3250;
    private double armSupLimit = 0;
    private double jointPos = 0;
    private boolean toggleSpeed = false;
    private boolean toggleBrake = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("IT", "FUCKING WORKED");
        telemetry.update();

        motor0 = hardwareMap.get(DcMotor.class, "MotorSF");
        motor1 = hardwareMap.get(DcMotor.class, "MotorDF");
        motor2 = hardwareMap.get(DcMotor.class, "MotorDS");
        motor3 = hardwareMap.get(DcMotor.class, "MotorSS");
        slider = hardwareMap.get(DcMotor.class, "Slider");
        arm    = hardwareMap.get(DcMotor.class,"Arm");
        armGem = hardwareMap.get(Servo.class, "ServoBratBila");
        clawL  = hardwareMap.get(Servo.class, "ServoClawL");
        clawR  = hardwareMap.get(Servo.class, "ServoClawR");
        cJoint= hardwareMap.get(Servo.class, "ClawJointL");

        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
        arm   .setDirection(DcMotor.Direction.FORWARD);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm   .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setTargetPosition(0);
        arm.setTargetPosition(0);
        armGem.setPosition(1);
        cJoint.setPosition(0.5);
        jointPos = 0.5;

        // Wait for the game to start (driver presses PLAY)
        runtime.reset();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //CADRU
            double sasiuPowerY = (-gamepad1.left_stick_y / 1.5) / speedDevider;
            double sasiuPowerX = (gamepad1.left_stick_x / 1.5) / speedDevider;
            double rotPower = 0.7;

            if(gamepad1.b && toggleBrake) {setBehaviour("Break");toggleBrake = false;sleep(100);}
            else if (gamepad1.b && !toggleBrake) {setBehaviour("Float");toggleBrake = true;sleep(100);}

            if(gamepad1.a && toggleSpeed) {speedDevider = 2.75; rotPower = 0.005;toggleSpeed = false;sleep(100);} //SlowMode ON
            else if(gamepad1.a && !toggleSpeed) { speedDevider = 1; rotPower = 0.7;toggleSpeed = true;sleep(100);} //SlowMode OFF

            if(gamepad1.left_stick_y == 0) {
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
            }

            if(-gamepad1.left_stick_y > 0.2 || -gamepad1.left_stick_y < -0.2) {
                motor0.setPower(-sasiuPowerY);
                motor1.setPower(sasiuPowerY);
                motor2.setPower(sasiuPowerY);
                motor3.setPower(-sasiuPowerY);
            }
            else if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_x < -0.2) {
                motor0.setPower(-sasiuPowerX);
                motor1.setPower(-sasiuPowerX);
                motor2.setPower(sasiuPowerX);
                motor3.setPower(sasiuPowerX);
            }

            if(gamepad1.left_bumper) {
                if(speedDevider == 1)   ///if SlowMode OFF
                {
                    motor0.setPower(rotPower);
                    motor1.setPower(rotPower);
                    motor2.setPower(rotPower);
                    motor3.setPower(rotPower);
                }
                else   //if SlowMode ON
                {
                    motor0.setPower(rotPower);
                    motor2.setPower(rotPower);
                }
            }
            if(gamepad1.right_bumper) {
                if(speedDevider == 1)      ///if SlowMode OFF
                {
                    motor0.setPower(-rotPower);
                    motor1.setPower(-rotPower);
                    motor2.setPower(-rotPower);
                    motor3.setPower(-rotPower);
                }
                else    //if SlowMode ON
                {
                     motor0.setPower(-rotPower);
                     motor2.setPower(-rotPower);
                }
            }

            //SLIDER  && slider.getCurrentPosition() < sliderSupLimit && slider.getCurrentPosition() > sliderInfLimit
            sliderInfLimit = 0;
            sliderSupLimit = 7700;
            if (-gamepad2.left_stick_y > 0.1  && slider.getCurrentPosition() < sliderSupLimit ||
                    gamepad2.left_stick_y > 0.1 && slider.getCurrentPosition() > sliderInfLimit)
                slider.setPower(-gamepad2.left_stick_y);
            else slider.setPower(0);

            //ARM && arm.getCurrentPosition() < armSupLimit && arm.getCurrentPosition() > armInfLimit
            armInfLimit = -3250;
            armSupLimit = 0;
            if (-gamepad2.right_stick_y > 0.1 && arm.getCurrentPosition() < armSupLimit ||
                    -gamepad2.right_stick_y < -0.1 && arm.getCurrentPosition() > armInfLimit)
                arm.setPower(-gamepad2.right_stick_y / 2);
            else arm.setPower(0);


            //CLAW
            if(gamepad2.left_bumper)
            {
                clawL.setPosition(-1);
                clawR.setPosition(1);
                sleep(50);
            }
            else if(gamepad2.right_bumper)
            {
                clawL.setPosition(1);
                clawR.setPosition(-1);
                sleep(50);
            }


            //JOINT
            if(gamepad2.left_trigger != 0 && jointPos < 0.84) {
                    jointPos = jointPos + 0.02;
                    //cJointR.setPosition(-jointPos);
                    cJoint.setPosition(jointPos);
                    sleep(50);
            }
            else if(gamepad2.right_trigger != 0 && jointPos > 0.18)
            {
                    jointPos = jointPos - 0.02;
                    //cJointR.setPosition(-jointPos);
                    cJoint.setPosition(jointPos);
                    sleep(50);
            }

            //RESET ARM AND SLIDER
            if(gamepad2.y)
            {
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setTargetPosition(0);
                slider.setPower(0.5);
                while(slider.isBusy() && opModeIsActive()) {
                    telemetry.addData("RESETING SLIDER", "");
                    telemetry.update();
                }
                slider.setPower(0);
                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if(slider.getCurrentPosition() >= -30 || slider.getCurrentPosition() <= 30) {
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setTargetPosition(0);
                    arm.setPower(0.5);
                    while(arm.isBusy() && opModeIsActive()) {
                        telemetry.addData("RESETING ARM", "");
                        telemetry.update();
                    }
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(arm.getCurrentPosition() >= -30 || arm.getCurrentPosition() <= 30)
                {   cJoint.setPosition(0.5);
                   // cJointR.setPosition(0);
                }
            }

            //CONTAINERS
            if(gamepad2.dpad_down) { //etaj 0
                slider.setTargetPosition(-2);
                arm.setTargetPosition(-3358);
                cJoint.setPosition(0.7);

                goToPos();
            }
            else if(gamepad2.dpad_left) { //etaj1
                slider.setTargetPosition(-2);
                arm.setTargetPosition(-2661);
                cJoint.setPosition(0.54);

                goToPos();
            }
            else if(gamepad2.dpad_up) { //etaj2
                slider.setTargetPosition(-2);
                arm.setTargetPosition(-1992);
                cJoint.setPosition(0.38);

                goToPos();
            }
            else if(gamepad2.dpad_right) { //etaj3
                slider.setTargetPosition(1288);
                arm.setTargetPosition(-1781);
                cJoint.setPosition(0.3);

                goToPos();
            }

            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Slider: ", slider.getCurrentPosition());
            telemetry.addData("Arm:", arm.getCurrentPosition());
            telemetry.addData("Motor0: ", motor0.getCurrentPosition());
            telemetry.addData("Motor1: ", motor1.getCurrentPosition());
            telemetry.addData("Motor2: ", motor2.getCurrentPosition());
            telemetry.addData("Motor3: ", motor3.getCurrentPosition());
            telemetry.addData("sasiuPowerX ", sasiuPowerX);
            telemetry.addData("sasiuPowerY: ", sasiuPowerY);
            telemetry.addData("clawL", clawL.getPosition());
            telemetry.addData("clawR", clawR.getPosition());
            telemetry.addData("jointPos", jointPos);
            telemetry.update();
        }
    }

    private void goToPos() {
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.4);
        while(slider.isBusy() && opModeIsActive()) {
            telemetry.addData("SLIDER MOVING", "");
            telemetry.addData("Slider: ", slider.getCurrentPosition());
            telemetry.addData("Arm: ", arm.getCurrentPosition());
            telemetry.addData("CJoint: ", cJoint.getPosition());
            telemetry.update();
        }
        slider.setPower(0);
        arm.setPower(0.4);
        while(arm.isBusy() && opModeIsActive()) {
            telemetry.addData("ARM MOVING", "");
            telemetry.addData("Slider: ", slider.getCurrentPosition());
            telemetry.addData("Arm: ", arm.getCurrentPosition());
            telemetry.addData("CJoint: ", cJoint.getPosition());
            telemetry.update();
        }
        arm.setPower(0);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setBehaviour(String str){
        if(str == "Break") {
            motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if(str == "Float"){
            motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}
