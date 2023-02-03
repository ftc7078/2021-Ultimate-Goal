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

import static org.firstinspires.ftc.teamcode.ButtonPressDetector.Button.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PoPTeleOp", group = "TeleOp")

public class PoPTeleOp extends LinearOpMode implements MecanumDrive.TickCallback {


private final MecanumDrive mecanumDrive = new MecanumDrive();

private final PoPRobot robot = new PoPRobot();
private boolean turretIsMoving = false;
private boolean elevatorIsMoving = false;


@Override
public void runOpMode() {
    robot.init(hardwareMap, telemetry, this);
    mecanumDrive.init(hardwareMap, telemetry, this);
    robot.setMotorDirections(mecanumDrive);
    mecanumDrive.setupTickCallback(this);
    ButtonPressDetector pad2pressDetector = new ButtonPressDetector(gamepad2);


    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");

    while (!isStarted()) {
        robot.getSleevePosition();
        sleep(50);
    }
    waitForStart();
    robot.stopVision();
    //START OF FFAuto
    // Auto Position 2 Fancy


    //robot.setDoorPosition(FFRobot.doorPosition.PICKUP);
    while (opModeIsActive()) {

        //MANIPULATOR

        if (pad2pressDetector.wasPressed(dpad_up)) {
            turnTurretTo(0);
        } else if (pad2pressDetector.wasPressed(dpad_down)) {
            turnTurretTo(180);
        } else if (pad2pressDetector.wasPressed(dpad_left)) {
            turnTurretTo(-90);
        } else if (pad2pressDetector.wasPressed(dpad_right)) {
            turnTurretTo(90);
        } else if (pad2pressDetector.wasPressed(dpad_up_left)) {
            turnTurretTo(-45);
        } else if (pad2pressDetector.wasPressed(dpad_up_right)) {
            turnTurretTo(45);
        } else if (pad2pressDetector.wasPressed(dpad_down_left)) {
            turnTurretTo(-135);
        } else if (pad2pressDetector.wasPressed(dpad_down_right)) {
            turnTurretTo(135);
        } else if (turretIsMoving) {
            if (!robot.turretTickResult() || gamepad2.back) {
                turretIsMoving = false;
                //robot.turretFreeMoveMode();
            }
        } else {
            robot.setTurretPower(gamepad2.left_stick_x);
        }


        if (gamepad2.a) {
            robot.turnArmTo(250);
            robot.setWrist(1);
        } else if (gamepad2.x) {
            robot.turnArmTo(120);
            robot.setWrist(0.5);
        } else if (gamepad2.b) {
            robot.turnArmTo(155);
            robot.setWrist(0.5);
        } else if (gamepad2.y) {
            robot.turnArmTo(90);
            robot.setWrist(0);
        } else if (gamepad2.start) {
            robot.turnArmTo(0);
            robot.setWrist(0);
        }

        if (elevatorIsMoving) {
            if (gamepad2.back) {
                robot.stopElevator();
                elevatorIsMoving = false;
            } else {
                if (robot.elevatorTickResult() == false) {
                    //done moving
                    elevatorIsMoving = false;
                }
                telemetry.addData("elevator power", "Auto");
            }
        } else {
            if (pad2pressDetector.wasPressed(right_stick_up)) {
                robot.setElevatorUp();
            } else if (pad2pressDetector.wasPressed(right_stick_down)) {
                robot.setElevatorDown();
            } else {
                robot.setElevatorPowerWithLimitSwitches(-gamepad2.right_stick_x);
                telemetry.addData("elevator power", -gamepad2.right_stick_x);
            }
        }


        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            robot.clawGrab();
        } else {
            robot.setClawPosition(gamepad2.left_trigger);
        }
        robot.setWristOffset(gamepad2.right_trigger / 3);

        standardMecanumControls();
        telemetry.update();

        telemetry.addData("turretIsMoving", turretIsMoving);
        telemetry.addData("turretPosition", robot.getTurretPosition());
        telemetry.addData("turretPower", robot.turret.getPower());
        telemetry.addData("Turret Position", robot.getTurretPosition());
        telemetry.addData("Arm Position", robot.getArmPosition());
        telemetry.addData("Elevator Position", robot.getElevatorPosition());
    }

}

public void standardMecanumControls() {
    double speed = 1;
    speed = (gamepad1.right_trigger * 0.5) + 0.5;
    double fwd = addDeadZone(gamepad1.left_stick_y) * speed;

    double rot = addDeadZone(gamepad1.right_stick_x) * speed;

    double strafe = addDeadZone(gamepad1.left_stick_x);
    strafe = strafe * speed * 1.6;
    if (strafe > 1) {
        strafe = 1;
    } else if (strafe < -1) {
        strafe = -1;
    }
    mecanumDrive.setMotors(strafe, fwd, rot, 1);
}

public void turnTurretTo(double degrees) {
    robot.turnTurretTo(-degrees, 1);
    turretIsMoving = true;
}

public void tickCallback() {
}

double addDeadZone(double input) {
    if (Math.abs(input) < 0.1) {
        return (0.0);
    }
    return (input);
}

}
