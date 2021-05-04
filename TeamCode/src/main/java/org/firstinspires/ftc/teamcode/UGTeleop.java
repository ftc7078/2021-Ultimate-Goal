

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="UG Teleop", group="Linear Opmode")

public class UGTeleop extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private boolean isSpeedUpPressed = false;
    private boolean isSpeedDownPressed = false;
    private boolean isShootTriggered = false;

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private UGRobot robot = new UGRobot();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap,telemetry,this);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            gamepad1.setJoystickDeadzone((float)0.2);
            double speed = 1;

             speed = (gamepad1.right_trigger * 0.6) + 0.4;
            double fwd = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rot= gamepad1.right_stick_x;

            fwd = fwd * speed;
            strafe =strafe * speed * 1.6;
            if (strafe > 1) {
                strafe = 1;
            } else if (strafe < -1) {
                strafe = -1;
            }
            rot = rot * speed;
            mecanumDrive.setMotors(strafe,fwd,rot, 1);

            boolean pull = gamepad2.x;
            boolean push = gamepad2.b;
            if (pull) {
                robot.setPickup(UGRobot.pickupDirection.IN);
                telemetry.addData("Manipulator Motors", "Pulling");
            } else if (push) {
                robot.setPickup(UGRobot.pickupDirection.OUT);
                telemetry.addData("Manipulator Motors", "Pushing");
            } else {
                telemetry.addData("Manipulator Motors", "Idle");
                robot.setPickup(UGRobot.pickupDirection.STOP);
            }


            boolean shootTriggered = gamepad2.right_bumper;

            if (gamepad2.right_bumper != isShootTriggered) {
                //mecanumDrive.setMotors(0,0,0, 1);
                if (gamepad2.right_bumper) {
                    robot.shoot();
                }
                isShootTriggered = gamepad2.right_bumper;
            }

            boolean speedUp = gamepad2.dpad_up;
            boolean speedDown = gamepad2.dpad_down;
            if (gamepad2.dpad_up != isSpeedUpPressed) {
                if (gamepad2.dpad_up) {
                    robot.setFlywheelPower(robot.getFlywheelPower()+0.02);
                    robot.setIdle(robot.getFlywheelPower()+0.02);
                    robot.setFlywheel(UGRobot.shooterDirection.OUT);
                }
                isSpeedUpPressed = gamepad2.dpad_up;
            }

            if (gamepad2.dpad_down != isSpeedDownPressed) {
                if (gamepad2.dpad_down) {
                    robot.setFlywheelPower(robot.getFlywheelPower()-0.02);
                    robot.setIdle(robot.getFlywheelPower()-0.02);
                    robot.setFlywheel(UGRobot.shooterDirection.OUT);
                }
                isSpeedDownPressed = gamepad2.dpad_down;
            }

            mecanumDrive.tickSleep();
            robot.tick();
            //telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Shoot Power", robot.getFlywheelPower());

            telemetry.addData("Current Velocity",robot.flyWheel.getCurrentVelocity());
            telemetry.addData("Desired Velocity", robot.flyWheel.getDesiredVelocity());
            telemetry.addData("MotorPower",robot.flyWheel.motor.getPower());
            telemetry.addData("change",robot.flyWheel.change);
            telemetry.addData("vs",robot.flyWheel.difference);

            telemetry.update();
        }

    }

}
