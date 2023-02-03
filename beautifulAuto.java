package org.firstinspires.ftc.teamcode;/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "beautifulAuto", group = "Concept")
public class beautifulAuto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 3;
    int MIDDLE = 1;
    int RIGHT = 4;

    AprilTagDetection tagOfInterest = null;

    private DcMotorEx LiftMotor1, LiftMotor2, angleMotor;
    private Servo intakeServo;
    private DcMotor FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    BNO055IMU imu;
    gyro Gyro;
    HolonomicDrive holonomicDrive;

    private int[] liftPos = {0, 1660, 2800, 4000};
    private int[] coneStack = {240, 265, 325, 370};
    int currentLiftPosition = 0;
    private int currentConePosition = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FrontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        BackRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        holonomicDrive = new HolonomicDrive(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor);
        Gyro = new gyro(FrontRightMotor, FrontLeftMotor, BackRightMotor, BackLeftMotor, imu);

        LiftMotor1 = hardwareMap.get(DcMotorEx.class, "LiftMotor1");
        LiftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMotor2");

        LiftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//      LiftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        LiftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //LiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LiftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LiftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        boolean done = false;

        if (opModeIsActive()) {
            while (opModeIsActive() & done != true) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                intakeServo.setPosition(0.0);
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < .25) {
                }
                if (currentDetections.size() != 0) {
                    boolean tagFound = false;
                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {

                        LiftMotor1.setTargetPosition(100);
                        LiftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        LiftMotor1.setPower(0.9);

                        LiftMotor2.setTargetPosition(100);
                        LiftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        LiftMotor2.setPower(0.9);

                        if (tagOfInterest.id == LEFT) {

                            // move forward

                            runtime.reset();
                            holonomicDrive.autoDrive(0, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 1.65) {
                            }

                            // intakeServo.setPosition(0.42);

                            // move left and park

                            runtime.reset();
                            holonomicDrive.autoDrive(270, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 1.25) {
                            }

                            holonomicDrive.autoDrive(0, 0);

                            done = true;

                        } else if (tagOfInterest.id == MIDDLE) {

                            // move forward

                            runtime.reset();
                            holonomicDrive.autoDrive(0, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 1.6) {
                            }

                            // move forward MORE (and back)

                            runtime.reset();
                            holonomicDrive.autoDrive(0, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 0.3) {
                            }

                            runtime.reset();
                            holonomicDrive.autoDrive(180, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 0.35) {
                            }

                            holonomicDrive.autoDrive(0, 0);

                            // intakeServo.setPosition(0.42);

                            done = true;

                        } else {

                            // move forward and right

                            runtime.reset();
                            holonomicDrive.autoDrive(0, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 1.65) {
                            }

                            runtime.reset();
                            holonomicDrive.autoDrive(90, 0.8);
                            while (opModeIsActive() && runtime.seconds() < 1) {
                            }

                            // intakeServo.setPosition(0.42);

                            holonomicDrive.autoDrive(0, 0);

                            done = true;

                        }


                    }

                }

            }

            LiftMotor1.setTargetPosition(0);
            LiftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LiftMotor1.setPower(0.9);

            LiftMotor2.setTargetPosition(0);
            LiftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LiftMotor2.setPower(0.9);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1.5) {
            }

            telemetry.addLine("Current Lift Motor Ticks: " + LiftMotor2.getCurrentPosition());
            telemetry.update();

        }
    }
}
