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


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private DcMotorEx turret =null;

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
        turret = hardwareMap.get(DcMotorEx.class, "turret");
    }
    public void setMotorDirections (MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);
    }
    void setTurretPower(double turretPowerIn){
        turret.setPower(turretPowerIn);
    }
    void stopVision() {
        try {
            camera.stopStreaming();
        } catch(Exception e) {

        }
    }

    int getSleevePosition () {
            AprilTagDetection bestTag = null;
            double bestTagRating = 0;
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
                    bestTagRating=bestTagRating-1;
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
                return(0);
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
                telemetry.addLine(output);
                telemetry.update();
                System.out.println(output);
                return (bestTag.id);
            }
        }
    }
