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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name = "MainAutonomous", group = "FTC")
//@Disabled
public class MainAutonomous extends LinearOpMode {

    private DcMotor motor0, motor1, motor2, motor3 = null;
    private Servo servoBratBila, clawL, clawR;
    private ColorSensor sensorBColor, sensorRColor;
    private DistanceSensor sensorBDistance, sensorRDistance;
    private String bilaColor, platformaColor;
    private VuforiaLocalizer vuforia;
    private DcMotor slider, arm = null;
    private Servo cJoint;
    private float y;

    @Override
    public void runOpMode() {

        motor0 = hardwareMap.get(DcMotor.class, "MotorSF");
        motor1 = hardwareMap.get(DcMotor.class, "MotorDF");
        motor2 = hardwareMap.get(DcMotor.class, "MotorDS");
        motor3 = hardwareMap.get(DcMotor.class, "MotorSS");
        sensorBColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorBila");
        sensorBDistance = hardwareMap.get(DistanceSensor.class, "Color/Range SensorBila");
        sensorRColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorRobot");
        sensorRDistance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Range SensorFront");
        servoBratBila = hardwareMap.get(Servo.class, "ServoBratBila");
        clawL = hardwareMap.get(Servo.class, "ServoClawL");
        clawR = hardwareMap.get(Servo.class, "ServoClawR");
        slider = hardwareMap.get(DcMotor.class, "Slider");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        cJoint = hardwareMap.get(Servo.class, "ClawJointL");

        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setTargetPosition(0);
        arm.setTargetPosition(0);
        cJoint.setPosition(0.5);

        servoBratBila.setPosition(1);

        clawL.setPosition(1);
        clawR.setPosition(-1);

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQoPNMX/////AAAAmad8NGRfnEv5lNS9s8cLm15ptksz2lDHtlK8+I/786kCdGjmFbNPW6iu9h7uJ1sXIChUyVAaSP0CD4ES7fguxYHIxlkuwz/MrOzyI9Wa2j5daMbcpXiHlGPYREiIqshHDjK13EGJtIVYfmI6d1JAoXmKbdQv43VDjyAJs4dYnHEdoBCalTdOX4KusyfQMckrkiutQnnHY9KHojBIEaQJTfKHEspulWitJBwkdLaWDBaXBlTekaa/aZyoZGsLsnW9lO7f/59KnS25gFyuuLLXWrJCnOipz+UyPB9dKJoEVIz6gvF2+rVyVIU6wMlgPP+e7LAVjP83S4qG8RK1CaRhnSfCMTE/0YC6Vuj0VSQwBGAX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
*/
        waitForStart();
        //relicTrackables.activate();

        while (opModeIsActive()) {
         /*   RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("RelicRecoveryVuMark", "LEFT");
                y = 1;
            }
            else if(vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("RelicRecoveryVuMark", "CENTER");
                y = 2;
            }
            else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("RelicRecoveryVuMark", "RIGHT");
                y = 3;
            }
            else {telemetry.addData("RelicRecoveryVuMark", "UNKNOWNNNNNNN");
                  y=3;}*/

            servoBratBila.setPosition(0.44);  ///////////////////////SERVO BRAT BILA
            sleep(2000);

           if(sensorBColor.red() > sensorBColor.blue())bilaColor = "Red";
           else bilaColor = "Blue";

           if(sensorRColor.red() > sensorRColor.blue())platformaColor = "Red";
           else platformaColor = "Blue";

            if(bilaColor == platformaColor) rotate(-500);
            else rotate(500);
            sleep(300);

                servoBratBila.setPosition(1);
                sleep(1000);

                if(platformaColor == bilaColor)rotate(330);
                else rotate(-100);

                if(platformaColor == "Blue")goBackwards(2900);
                else goForward(2900);
                sleep(30000);

            /*if(stoneColor == "Red")if(y == 3)y = 1;
                                   else if(y == 1)y = 3;

            if(stoneColor == "Blue")if(y == 1)goSideways(500);
                               else if (y == 2)goSideways(1000);
                                    else goSideways(1500);
            else if(y == 1)goSideways(-500);
                 else if (y == 2)goSideways(-1000);
                      else goSideways(-1500);*/



                }
                /// slider 3240   arm 65   joint 0.7
            }

    private void goForward(int x) {
        motor0.setTargetPosition(-x);
        motor1.setTargetPosition(x);
        motor2.setTargetPosition(x);
        motor3.setTargetPosition(-x);

            motor0.setPower(0.4);//was 0.4
            motor1.setPower(0.4);//was 0.5
            motor2.setPower(0.4);//was 0.4
            motor3.setPower(0.4);//was 0.5

                while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if (Math.abs(x - motor0.getCurrentPosition()) < 50) {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.2);
            }
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    private void goBackwards(int x) {
        motor0.setTargetPosition(x);
        motor1.setTargetPosition(-x);
        motor2.setTargetPosition(-x);
        motor3.setTargetPosition(x);

        motor0.setPower(0.4);
        motor1.setPower(0.4);
        motor2.setPower(0.4);
        motor3.setPower(0.4);

        while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if (Math.abs(x - motor0.getCurrentPosition()) < 50) {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.2);
            }
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    private void goSideways(int x){
        motor0.setTargetPosition(x);
        motor1.setTargetPosition(x);
        motor2.setTargetPosition(-x);
        motor3.setTargetPosition(-x);

        motor0.setPower(0.4);
        motor1.setPower(0.4);
        motor2.setPower(0.4);
        motor3.setPower(0.4);

        while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if(Math.abs(x - motor0.getCurrentPosition()) < 50 )
            {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.2);
            }
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }

    private void rotate(int x){
        motor0.setTargetPosition(x);
        motor1.setTargetPosition(x);
        motor2.setTargetPosition(x);
        motor3.setTargetPosition(x);

        motor0.setPower(0.3);
        motor1.setPower(0.3);
        motor2.setPower(0.3);
        motor3.setPower(0.3);

        while (motor0.isBusy() && opModeIsActive()) {
            if(Math.abs(x - motor0.getCurrentPosition()) < 50 )
            {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.2);
            }
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
}
