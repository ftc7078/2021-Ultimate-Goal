package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class PoPRobot {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Telemetry telemetry = null;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    final int TURRET_COUNT_PER_DEGREE = 135;
    final int TURN_LIMIT = TURRET_COUNT_PER_DEGREE * 180;
    final double ARM_COUNT_PER_DEGREE = 4200 / 360;
    final double CLAW_CLOSED = 1;
    final double CLAW_OPEN = 0;
    final int ELEVATOR_TOLERANCE = 10;
    final int ELEVATOR_UP_POSITION = 1850;
    final int ELEVATOR_DOWN_POSITION = 0;

    DcMotorEx turret =null;
    private DcMotorEx arm =null;
    private Servo claw =null;
    private DcMotorEx elevator =null;
    private Servo wrist =null;
    double wristBasePosition=0;
    AprilTagDetection bestTag = null;
    double bestTagRating = 0;

    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {
        telemetry = telemetryIn;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
        turret = hardwareMap.get(DcMotorEx.class, "turrets");
        //turret = new HPMC(hardwareMap,"turrets",24000);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        wrist = hardwareMap.get(Servo.class, "wrist");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PIDCoefficients pidf = turret.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        //pidf.p=1000;
        //pidf.i=100;
        //pidf.d=0;
        //turret.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidf);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getArmPosition(){
        return arm.getCurrentPosition();
    }

    public void setMotorDirections (MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(REVERSE, FORWARD, FORWARD, REVERSE);
    }

    public void setTurretPower(double turretPowerIn){

        if ((turret.getCurrentPosition() < -24000) && (turretPowerIn < 0) ) {
            turret.setPower(0);
        } else if ( (turret.getCurrentPosition()) > 24000 && (turretPowerIn>0) )  {
            turret.setPower(0);
        } else {
            turret.setPower(turretPowerIn);
        }

    }

    public int getTurretPosition(){
        return turret.getCurrentPosition();
    }


    public void turnTurret(double degrees){
        int destination = getTurretPosition() + (int) (degrees * TURRET_COUNT_PER_DEGREE);
        if (destination < -TURN_LIMIT) {
            destination = -TURN_LIMIT;
        } else if (destination > TURN_LIMIT) {
            destination = TURN_LIMIT;
        }
        turret.setTargetPositionTolerance(TURRET_COUNT_PER_DEGREE);
        //turret.smoothMoveSetup(destination-turret.getCurrentPosition(), 0.6, 3,3, HPMC.Direction.FORWARD, true);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }


    public void turnTurretTo(double degrees){
        if (degrees > 180) {
            degrees = 180;
        } else if (degrees < -180) {
            degrees = -180;
        }
        int destination = (int) (TURRET_COUNT_PER_DEGREE * degrees);
        //turret.smoothMoveSetup(destination-turret.getCurrentPosition(), 0.8, 3,3, HPMC.Direction.FORWARD, true);
        turret.setPower(0);
        turret.setTargetPositionTolerance(TURRET_COUNT_PER_DEGREE);

    }

    //public boolean isTurretAtDestination() {
     //   return (Math.abs(turret.getCurrentPosition() - turret.getTargetPosition()) < TURRET_COUNT_PER_DEGREE);
    //}
    //public int turretLeftToDestination() {
    //    return (turret.getCurrentPosition() - turret.getTargetPosition());
    //}
    //public DcMotorEx getTurret() {
    //    return turret;
    //}

    //public void turretFreeMoveMode() {
     //   turret.setPower(0);
     //   turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //}

    public void turnArmTo (double degrees){
        if (degrees > 250) {
            degrees = 250;
        } else if (degrees < 0) {
            degrees = 0;
        }
        int destination = (int) (ARM_COUNT_PER_DEGREE * degrees);
        arm.setTargetPositionTolerance((int) ARM_COUNT_PER_DEGREE);
        arm.setTargetPosition(destination);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
    }

    /*
    public boolean turretStillMoving() {
        turret.updateCurrentVelocity();
        return turret.smTick();
    }
     */
    public void setWrist (double position){
        wristBasePosition=position;
    }
    public void setWristOffset(double offset) {
        wrist.setPosition(wristBasePosition+offset);
    }

    public boolean isArmAtDestination() {
        return (Math.abs(arm.getCurrentPosition() - arm.getTargetPosition()) < ARM_COUNT_PER_DEGREE);
    }

    public void clawGrab(){
        claw.setPosition(CLAW_CLOSED);
    }

    public void clawRelease(){
        claw.setPosition(CLAW_OPEN);
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }


    public void setElevatorUp() {
        setElevatorPosition(ELEVATOR_UP_POSITION);
    }
    public void setElevatorDown() {
        setElevatorPosition(ELEVATOR_DOWN_POSITION);
    }
    public void setElevatorPosition(int destination) {
        elevator.setPower(0);
        elevator.setTargetPositionTolerance(ELEVATOR_TOLERANCE);
        elevator.setTargetPosition(destination);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);

    }
    public void setElevatorPower(double power) {
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setPower(power);
    }
    public void setElevatorFreeMove() {
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getElevatorPosition() {
        return elevator.getCurrentPosition();
    }
    public void stopVision() {
        try {
            camera.stopStreaming();
        } catch(Exception e) {
            //ignore
        }
    }

    public int getSleevePosition () {

            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }
                    bestTagRating=bestTagRating-0.1;
                    if (bestTagRating < 0) {
                        bestTagRating = 0;
                        bestTag=null;
                    }
                    for(AprilTagDetection detection : detections) {
                        double thisTagRating = 0;
                        if ((detection.id > 0) && (detection.id < 7)) {
                            thisTagRating = 10 / (1 +
                                    Math.abs(detection.pose.x) +
                                    Math.abs(detection.pose.y) +
                                    (Math.abs(Math.toDegrees(detection.pose.yaw) + 12) / 45) +
                                    (Math.abs(Math.toDegrees(detection.pose.pitch)) / 45) +
                                    (Math.abs(Math.toDegrees(detection.pose.roll)) / 45)
                            );


                        }
                        if (thisTagRating > bestTagRating) {
                            bestTag=detection;
                            bestTagRating=thisTagRating;
                        }
                    }
                }
            }
            if (bestTag == null) {
                System.out.println ( "Best tag was null");
                return(-1);
            } else {
                String output = String.format("R: %.2f ID=%d XYZ: %.2f-%.2f-%.2f YPR: %.2f:%.2f:%.2f",
                        bestTagRating,
                        bestTag.id,
                        bestTag.pose.x * FEET_PER_METER,
                        bestTag.pose.y * FEET_PER_METER,
                        bestTag.pose.z * FEET_PER_METER,
                        Math.toDegrees(bestTag.pose.yaw),
                        Math.toDegrees(bestTag.pose.pitch),
                        Math.toDegrees(bestTag.pose.roll)
                );
                //telemetry.addLine(output);
                //telemetry.update();
                System.out.println(output);
                return (bestTag.id);
            }
        }
    }
