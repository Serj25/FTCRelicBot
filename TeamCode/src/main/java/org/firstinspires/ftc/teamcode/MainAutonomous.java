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
    private String glyphColor, stoneColor;
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQoPNMX/////AAAAmad8NGRfnEv5lNS9s8cLm15ptksz2lDHtlK8+I/786kCdGjmFbNPW6iu9h7uJ1sXIChUyVAaSP0CD4ES7fguxYHIxlkuwz/MrOzyI9Wa2j5daMbcpXiHlGPYREiIqshHDjK13EGJtIVYfmI6d1JAoXmKbdQv43VDjyAJs4dYnHEdoBCalTdOX4KusyfQMckrkiutQnnHY9KHojBIEaQJTfKHEspulWitJBwkdLaWDBaXBlTekaa/aZyoZGsLsnW9lO7f/59KnS25gFyuuLLXWrJCnOipz+UyPB9dKJoEVIz6gvF2+rVyVIU6wMlgPP+e7LAVjP83S4qG8RK1CaRhnSfCMTE/0YC6Vuj0VSQwBGAX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        float hsvValues[] = {0F, 0F, 0F};
        float hsvValues2[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final float values2[] = hsvValues2;
        final double SCALE_FACTOR = 255;

        waitForStart();
        relicTrackables.activate();

        while (opModeIsActive()) {

            goForward(2727);
            sleep(30000);

            arm.setTargetPosition(65);
            slider.setTargetPosition(3240);
            arm.setPower(0.3);
            slider.setPower(0.3);
            while (arm.isBusy() && opModeIsActive()) {
                if (Math.abs(65 - arm.getCurrentPosition()) < 50) {
                    arm.setPower(0.2);
                }
                while (slider.isBusy() && opModeIsActive()) {
                    if (Math.abs(3240 - slider.getCurrentPosition()) < 50) {
                        slider.setPower(0.2);
                    }
                    arm.setPower(0);
                    slider.setPower(0);

                    cJoint.setPosition(0.7);
                    sleep(100);
                    clawL.setPosition(-1);
                    clawR.setPosition(1);

                    //rotate(100);

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
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
                  y=3;}



            telemetry.addData("Coboram Bratul", "");
            telemetry.update();
            sleep(300);
            servoBratBila.setPosition(0.48);  ///////////////////////SERVO BRAT BILA

            telemetry.addData("Scanam Culoarea", "");
            telemetry.update();

            Color.RGBToHSV((int) (sensorBColor.red() * SCALE_FACTOR),
                    (int) (sensorBColor.green() * SCALE_FACTOR),
                    (int) (sensorBColor.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int)  (sensorRColor.red() * SCALE_FACTOR),
                    (int)  (sensorRColor.green() * SCALE_FACTOR),
                    (int) (sensorRColor.blue() * SCALE_FACTOR),
                    hsvValues2);
            sensorBColor.enableLed(true);
            sensorRColor.enableLed(true);

            if(hsvValues[0] < 50)
                glyphColor = "Red";
            else glyphColor = "Blue";

            if(hsvValues2[0] < 50)
                stoneColor = "Red";
            else stoneColor = "Blue";

            telemetry.addData("GlyphCOLOR: ", glyphColor);
            telemetry.addData("StoneCOLOR: ", stoneColor);
            telemetry.update();
            sleep(200);

                    //goForward(2500);
                    sleep(500);

            /*if(stoneColor == "Red")if(y == 3)y = 1;
                                   else if(y == 1)y = 3;

            if(stoneColor == "Blue")if(y == 1)goSideways(500);
                               else if (y == 2)goSideways(1000);
                                    else goSideways(1500);
            else if(y == 1)goSideways(-500);
                 else if (y == 2)goSideways(-1000);
                      else goSideways(-1500);*/

                    sleep(20000);

           /* if(stoneColor != glyphColor)
                telemetry.addData("Going", "FORWARD");
            else telemetry.addData("Going", "BACKWARDS");
            telemetry.update();
            sleep(200);

            if(stoneColor != glyphColor) goForward(20);
            else goBackwards(20);
            sleep(1000);

            servoBratBila.setPosition(1);

            goForward(2300); //Mergi inainte o placa
            sleep(10000);*/

//            telemetry.addData("Color: ", glyphColor);
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", sensorBColor.alpha());
//            telemetry.addData("Red  ", sensorBColor.red());
//            telemetry.addData("Green", sensorBColor.green());
//            telemetry.addData("Blue ", sensorBColor.blue());
//            telemetry.addData("Hue", hsvValues[0]);

                    telemetry.update();
                }
                /// slider 3240   arm 65   joint 0.7
            }
        }
    }

    private void goForward(int x) {
       /* double initdist;
        initdist = sensorRDistance.getDistance(DistanceUnit.CM);
        while ((initdist-sensorRDistance.getDistance(DistanceUnit.CM))>x) {
            telemetry.addData("Distanta Masurata: ", "%f cm", sensorRDistance.getDistance(DistanceUnit.CM));
            telemetry.update();*/
        motor0.setTargetPosition(-x);
        motor1.setTargetPosition(x);
        motor2.setTargetPosition(x);
        motor3.setTargetPosition(-x);

            motor0.setPower(0.4);//was 0.4
            motor1.setPower(0.4);//was 0.5
            motor2.setPower(0.4);//was 0.4
            motor3.setPower(0.03);//was 0.5

                while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if (Math.abs(x - motor0.getCurrentPosition()) < 50) {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.01);
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
        motor3.setPower(0.03);

        while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if (Math.abs(x - motor0.getCurrentPosition()) < 50) {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.01);
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
        motor3.setPower(0.03);

        while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
            if(Math.abs(x - motor0.getCurrentPosition()) < 50 )
            {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.01);
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
        motor3.setPower(0.03);

        while (motor0.isBusy() && opModeIsActive()) {
            if(Math.abs(x - motor0.getCurrentPosition()) < 50 )
            {
                motor0.setPower(0.2);
                motor1.setPower(0.2);
                motor2.setPower(0.2);
                motor3.setPower(0.02);
            }
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
}
