/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    private DcMotor shooter = null;
    private Servo launchServo;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double idle = 0.52;
    private double shoot = 0.52;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {IN, OUT, STOP, IDLE}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"servo");

        pickupbottom = hardwareMap.get(DcMotor.class, "em1");
        pickuptop = hardwareMap.get(DcMotor.class, "em2");
        shooter = hardwareMap.get(DcMotor.class,"em0");

        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setShooter(shooterDirection.IDLE);
        setLaunchServo(false);


    }

    public void shoot (boolean out) {
        if (out) {
            setShooter(UGRobot.shooterDirection.OUT);
            opMode.sleep(500);
            setLaunchServo(true);
            opMode.sleep(300);
            setLaunchServo(false);
            opMode.sleep(700);
        } else {
            setShooter(shooterDirection.IDLE);
        }

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

    public double getShoot() {
        return shoot;
    }

    public void setShoot(double shoot) {
        this.shoot = shoot;
    }

    public void setShooter(UGRobot.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case IN:
                shooter.setPower(-1);
                break;
            case OUT:
                shooter.setPower(shoot);
                break;
            case STOP:
                shooter.setPower(0);
                break;
            case IDLE:
                shooter.setPower(idle);
                break;
        }
    }
}