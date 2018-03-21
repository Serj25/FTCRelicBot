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
    private Servo servoBratBila;
    private ColorSensor sensorBColor, sensorRColor;
    private DistanceSensor sensorBDistance, sensorRDistance;
    private String glyphColor, stoneColor;
    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        motor0 = hardwareMap.get(DcMotor.class, "MotorSF");
        motor1 = hardwareMap.get(DcMotor.class, "MotorDF");
        motor2 = hardwareMap.get(DcMotor.class, "MotorDS");
        motor3 = hardwareMap.get(DcMotor.class, "MotorSS");
        sensorBColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorBila");
        sensorBDistance = hardwareMap.get(DistanceSensor.class, "Color/Range SensorBila");
        sensorRColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorRobot");
        sensorRDistance = hardwareMap.get(DistanceSensor.class, "Color/Range SensorRobot");
        servoBratBila = hardwareMap.get(Servo.class, "ServoBratBila");

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

        servoBratBila.setPosition(1);

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

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark == RelicRecoveryVuMark.LEFT)
                telemetry.addData("RelicRecoveryVuMark", "LEFT");
            else if(vuMark == RelicRecoveryVuMark.CENTER)
                telemetry.addData("RelicRecoveryVuMark", "CENTER");
            else if(vuMark == RelicRecoveryVuMark.RIGHT)
                telemetry.addData("RelicRecoveryVuMark", "RIGHT");
            else telemetry.addData("RelicRecoveryVuMark", "UNKNOWNNNNNNN");



            telemetry.addData("Coboram Bratul", "");
            telemetry.update();
            sleep(1000);
            servoBratBila.setPosition(0.3);

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
            sleep(1000);

            if(stoneColor != glyphColor)
                telemetry.addData("Going", "FORWARD");
            else telemetry.addData("Going", "BACKWARDS");
            telemetry.update();
            sleep(1000);

            if(stoneColor != glyphColor) goForward();
            else goBackwards();


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
    }
    void goForward() {
        motor0.setTargetPosition(-200);
        motor1.setTargetPosition(200);
        motor2.setTargetPosition(200);
        motor3.setTargetPosition(-200);

        motor0.setPower(0.3);
        while (motor0.isBusy() && opModeIsActive()) {
            if(Math.abs(200 - motor0.getCurrentPosition()) < 50 )
                motor0.setPower(0.2);
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
    void goBackwards() {
        motor0.setTargetPosition(200);
        motor1.setTargetPosition(-200);
        motor2.setTargetPosition(-200);
        motor3.setTargetPosition(200);

        motor0.setPower(0.3);
        while (motor0.isBusy() && opModeIsActive()) {
            if(Math.abs(200 - motor0.getCurrentPosition()) < 50 )
                motor0.setPower(0.2);
        }
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
    }
}
