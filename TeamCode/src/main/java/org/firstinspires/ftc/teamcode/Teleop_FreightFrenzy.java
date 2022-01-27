

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="FreightFrenzy_Teleop", group="Linear Opmode")

public class Teleop_FreightFrenzy extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private FFRobot robot = new FFRobot();

    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        robot.init(hardwareMap,telemetry,this);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double speed = 1;

             speed = (gamepad1.right_trigger * 0.3) + 0.7;
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

            if (gamepad1.dpad_up) {
                robot.moveArm(FFRobot.armPosition.HIGH);
            }
            if (gamepad1.dpad_left) {
                robot.moveArm(FFRobot.armPosition.MIDDLE);
            }
            if (gamepad1.dpad_right) {
                robot.moveArm(FFRobot.armPosition.MIDDLE);
            }
            if (gamepad1.dpad_down) {
                robot.moveArm(FFRobot.armPosition.LOW);
            }
            if (gamepad1.a) {
                robot.moveArm(FFRobot.armPosition.PICKUP);
            }


            if (gamepad1.right_bumper) {
                robot.arm.setPower(0);
                robot.arm.setTargetPosition(robot.arm.getCurrentPosition());
            }
           if (gamepad1.b) {
               robot.pickup(true);
           } else {
               robot.pickup(false);

           }
            if (gamepad1.left_bumper) {
                robot.setDuckWheel(0.7);
            } else if(gamepad1.right_bumper) {
                robot.setDuckWheel(-0.7);
            } else {
                robot.setDuckWheel(0);
            }
            robot.setShippingElementPickupPosition(gamepad1.left_trigger);





            mecanumDrive.tickSleep();
            telemetry.addData("gp1lt", gamepad1.left_trigger);
            //telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            telemetry.addData("Motor Power",robot.arm.getPower());
            telemetry.addData("Motor Position",robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }

}
