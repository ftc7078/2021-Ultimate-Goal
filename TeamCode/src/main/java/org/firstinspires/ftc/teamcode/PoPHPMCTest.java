/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="PoPTurretTestHPMC", group ="TeleOp")

public class PoPHPMCTest extends LinearOpMode  {
    private static final int TURRET_COUNT_PER_DEGREE = 135;
    HPMC turret = null;
    @Override
    public void runOpMode() {
        turret = new HPMC(hardwareMap,"turrets",2800);
        waitForStart();
        turret.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setLabel("turret");

        DetectOnce dpadUpOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_up);
        DetectOnce dpadDownOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_down);
        DetectOnce dpadLeftOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_left);
        DetectOnce dpadRightOnce = new DetectOnce(gamepad2, DetectOnce.Button.dpad_right);
        DetectOnce leftBumperOnce = new DetectOnce(gamepad2, DetectOnce.Button.left_bumper);
        DetectOnce rightBumperOnce = new DetectOnce(gamepad2, DetectOnce.Button.right_bumper);
        DetectOnce startOnce = new DetectOnce(gamepad2, DetectOnce.Button.start);
        DetectOnce backOnce = new DetectOnce(gamepad2, DetectOnce.Button.back);
        DetectOnce aOnce = new DetectOnce(gamepad2, DetectOnce.Button.a);
        DetectOnce xOnce = new DetectOnce(gamepad2, DetectOnce.Button.x);
        DetectOnce bOnce = new DetectOnce(gamepad2, DetectOnce.Button.b);

        int decelerationTicks = 15;

        while (opModeIsActive()) {
            if (dpadUpOnce.pressed() ) {
                decelerationTicks++;
            }
            if (dpadDownOnce.pressed()) {
                decelerationTicks--;
            }
            if (dpadLeftOnce.pressed()) {
            }
            if (dpadRightOnce.pressed()) {
            }
            if (leftBumperOnce.pressed()) {
            }
            if (rightBumperOnce.pressed()) {
            }
            if (startOnce.pressed()) {
            }
            if (backOnce.pressed()) {
            }

            if (aOnce.pressed()) {
                turret.runToPosition(350, 1,decelerationTicks);
            }
            if (xOnce.pressed()) {
                turret.runToPosition(0, 1, decelerationTicks);
            }
            if (bOnce.pressed()) {
                turret.stop();
            }
            turret.onTick();
            telemetry.addData("Deceleration Ticks",decelerationTicks);
            telemetry.addData("Position",turret.getCurrentPosition());
            telemetry.update();
            sleep(50);

        }

    }
    public void turnTurretTo(double degrees){
        if (degrees > 180) {
            degrees = 180;
        } else if (degrees < -180) {
            degrees = -180;
        }
        int destination = (int) (TURRET_COUNT_PER_DEGREE * degrees);
    }

    private boolean isTurretAtDestination() {
        return (Math.abs(getTurretLeft()) < 150);
    }
    private int getTurretLeft() {
        return (turret.getTargetPosition() - turret.getCurrentPosition());
    }

}
