

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name="Test: Just Mecanum", group="Test")

public class JustMecanum extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    /*
    private DcMotor rightManipulator = null;
    //private DcMotor leftManipulator = null;
    private final int FL=0;
    private final int FR=1;
    private final int BL =2;
    private final int BR =3;
    private final double SLOW=0.4;
    private final double MAX_SPEED=2800;
    Servo capstone;
    Servo foundationRight;
    Servo foundationLeft;
    double capstonePosition=0;
    private double mSpeed=1.0;

     */

    private MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap, telemetry, this);
        mecanumDrive.setMotorDirections(Direction.FORWARD, Direction.REVERSE, Direction.FORWARD, Direction.REVERSE);


        //capstone = hardwareMap.get(Servo.class, "capstone");
        //foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        //foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        int j=100;
        while (opModeIsActive()) {

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

            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                if (gamepad1.a) {
                    mecanumDrive.runMotors(0.5,0,0,0,false);
                    telemetry.addData("Running", "Front Left");
                } else if (gamepad1.b) {
                    mecanumDrive.runMotors(0,0.5,0,0,false);
                    telemetry.addData("Running", "Front Right");
                } else if (gamepad1.x) {

                    mecanumDrive.runMotors(0,0,0.5,0,false);
                    telemetry.addData("Running", "Back Left");
                } else if (gamepad1.y) {
                    mecanumDrive.runMotors(0,0,0,0.5,false);
                    telemetry.addData("Running", "Back Right");
                }
            } else {
                mecanumDrive.setMotors(strafe, fwd, rot, 1);
            }

            telemetry.addData("Left/Right Stick", "LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
            mecanumDrive.motorTelemetry();

            j++;
            telemetry.addData("Number:", "%d", j);
            telemetry.update();
            mecanumDrive.tickSleep();
        }

    }

}
