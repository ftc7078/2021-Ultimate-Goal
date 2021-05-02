/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UGRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;

    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    private HPMC flyWheel = null;
    private HPMC wobbleArmMotor = null;
    private Servo launchServo;
    private Servo gripper;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double idle = 0.61;
    private double shooterPower = 0.61;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {IN, OUT, STOP, IDLE}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"launchServo");
        gripper = hardwareMap.get(Servo.class,"gripper");

        pickupbottom = hardwareMap.get(DcMotor.class, "pickupBottom");
        pickuptop = hardwareMap.get(DcMotor.class, "pickupTop");
        wobbleArmMotor = new HPMC(hardwareMap, "wobble", 3000);
        flyWheel = new HPMC(hardwareMap,"shooter",3000);

        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setShooter(shooterDirection.IDLE);
        setLaunchServo(false);


    }

    public void shoot (boolean out) {
        if (out) {
            setShooter(UGRobot.shooterDirection.OUT);
            setLaunchServo(true);
            opMode.sleep(300);
            setLaunchServo(false);
            opMode.sleep(700);
        } else {
            setShooter(shooterDirection.IDLE);
        }

    }

    public double findShooterSpeed () {
        flyWheel.updateCurrentVelocity();
        return (flyWheel.getCurrentVelocity());
    }

    public void setPickup(UGRobot.pickupDirection direction) {
        pickupState = direction;
        switch (direction) {
            case IN:
                pickupbottom.setPower(-1);
                pickuptop.setPower(-1);
                break;
            case OUT:
                pickupbottom.setPower(1);
                pickuptop.setPower(1);
                break;
            case STOP:
                pickupbottom.setPower(0);
                pickuptop.setPower(0);
                break;
        }
    }
    public void setLaunchServo (boolean in) {
        if(in) {
            launchServo.setPosition(0);
        } else {
            launchServo.setPosition(1);
        }
    }

    public double getIdle() {
        return idle;
    }

    public void setIdle(double idle) {
        this.idle = idle;
    }

    public double getShooterPower() {
        return shooterPower;
    }

    public void setShooterPower(double shooterPower) {
        this.shooterPower = shooterPower;
    }

    public int getShooterEncoderPosition() {
        return flyWheel.motor.getCurrentPosition();
    }

    public void setShooter(UGRobot.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case IN:
                flyWheel.setPowerManual(-1);
                break;
            case OUT:
                flyWheel.setPowerManual(shooterPower);
                break;
            case STOP:
                flyWheel.setPowerManual(0);
                break;
            case IDLE:
                flyWheel.setPowerManual(idle);
                break;
        }
    }
}