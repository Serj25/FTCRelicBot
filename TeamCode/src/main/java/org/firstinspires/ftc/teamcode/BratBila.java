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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "REVColorDistance", group = "Sensor")
//@Disabled
public class BratBila extends LinearOpMode {

    Servo servoBratBila;
    ColorSensor sensorBColor, sensorRColor;
    DistanceSensor sensorBDistance, sensorRDistance;
    String glyphColor, stoneColor;

    @Override
    public void runOpMode() {

        sensorBColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorBila");
        sensorBDistance = hardwareMap.get(DistanceSensor.class, "Color/Range SensorBila");
        sensorRColor = hardwareMap.get(ColorSensor.class, "Color/Range SensorRobot");
        sensorRDistance = hardwareMap.get(DistanceSensor.class, "Color/Range SensorRobot");

        servoBratBila = hardwareMap.get(Servo.class, "ServoBratBila");

        float hsvValues[] = {0F, 0F, 0F};
        float hsvValues2[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final float values2[] = hsvValues2;
        final double SCALE_FACTOR = 255;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Coboram Bratul", "");
            telemetry.update();
            sleep(1000);
            servoBratBila.setPosition(-0.5);

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

            if(stoneColor != glyphColor)


//            telemetry.addData("Color: ", glyphColor);
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
//            telemetry.addData("Alpha", sensorBColor.alpha());
//            telemetry.addData("Red  ", sensorBColor.red());
//            telemetry.addData("Green", sensorBColor.green());
//            telemetry.addData("Blue ", sensorBColor.blue());
//            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}
