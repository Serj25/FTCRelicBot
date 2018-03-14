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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="VuMarkCheck2", group ="Concept")
//@Disabled
public class VuMarkCheck2 extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    private ElapsedTime runtime = new ElapsedTime();
    private OpenGLMatrix lastLocation = null;
    private DcMotor motor0, motor1, motor2, motor3 = null;
    private int targetPos;
    private VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        motor0 = hardwareMap.get(DcMotor.class, "MotorSF");
        motor1 = hardwareMap.get(DcMotor.class, "MotorDF");
        motor2 = hardwareMap.get(DcMotor.class, "MotorDS");
        motor3 = hardwareMap.get(DcMotor.class, "MotorSS");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQoPNMX/////AAAAmad8NGRfnEv5lNS9s8cLm15ptksz2lDHtlK8+I/786kCdGjmFbNPW6iu9h7uJ1sXIChUyVAaSP0CD4ES7fguxYHIxlkuwz/MrOzyI9Wa2j5daMbcpXiHlGPYREiIqshHDjK13EGJtIVYfmI6d1JAoXmKbdQv43VDjyAJs4dYnHEdoBCalTdOX4KusyfQMckrkiutQnnHY9KHojBIEaQJTfKHEspulWitJBwkdLaWDBaXBlTekaa/aZyoZGsLsnW9lO7f/59KnS25gFyuuLLXWrJCnOipz+UyPB9dKJoEVIz6gvF2+rVyVIU6wMlgPP+e7LAVjP83S4qG8RK1CaRhnSfCMTE/0YC6Vuj0VSQwBGAX";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("Test");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        RelicRecoveryVuMark lastVuMark = RelicRecoveryVuMark.UNKNOWN;

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            telemetry.addData("VuMark", "%s is still the same", vuMark);
            telemetry.update();

            if (vuMark != lastVuMark) {

                telemetry.addData("VuMark", "%s != %s", vuMark, lastVuMark);
                telemetry.update();

                sleep(1000);

                lastVuMark = vuMark;

                if(vuMark == RelicRecoveryVuMark.LEFT) targetPos = 500;
                else if(vuMark == RelicRecoveryVuMark.CENTER) targetPos = 1000;
                else if(vuMark == RelicRecoveryVuMark.RIGHT) targetPos = 1500;

                motor0.setTargetPosition(motor0.getCurrentPosition() - targetPos);
                motor1.setTargetPosition(motor1.getCurrentPosition() + targetPos);
                motor2.setTargetPosition(motor2.getCurrentPosition() + targetPos);
                motor3.setTargetPosition(motor3.getCurrentPosition() - targetPos);

                setAllMotorPower(0.5);

                while (motor0.isBusy() && motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && opModeIsActive()) {
                    if (Math.abs(targetPos - motor0.getCurrentPosition()) < 50)
                        setAllMotorPower(0.2);
                    showData();
                }

                setAllMotorPower(0);

                sleep(2000);
            }
            if(vuMark == RelicRecoveryVuMark.UNKNOWN)
                telemetry.addData("UNKNOWN Pictogram", " ");

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    private void setAllMotorPower(double power) {
        motor0.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
    }

    private void showData() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("TargetPos: ", targetPos);
        telemetry.addData("Motor0: ", motor0.getCurrentPosition());
        telemetry.addData("Motor1: ", motor1.getCurrentPosition());
        telemetry.addData("Motor2: ", motor2.getCurrentPosition());
        telemetry.addData("Motor3: ", motor3.getCurrentPosition());


        telemetry.update();

    }
}
