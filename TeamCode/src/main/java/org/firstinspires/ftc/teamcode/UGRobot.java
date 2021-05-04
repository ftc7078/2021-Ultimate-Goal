/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class UGRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;

    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    FlywheelMC flyWheel = null;
    private HPMC wobbleArmMotor = null;
    private Servo launchServo;
    private Servo gripper;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double idle = 0.61;
    private double flywheelPower = 0.61;
    private ArrayList<Long> toggleQueue = new ArrayList<Long>();
    private boolean launchServoState;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {OUT, STOP}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"launchServo");
        gripper = hardwareMap.get(Servo.class,"gripper");

        pickupbottom = hardwareMap.get(DcMotor.class, "pickupBottom");
        pickuptop = hardwareMap.get(DcMotor.class, "pickupTop");
        wobbleArmMotor = new HPMC(hardwareMap, "wobble", 3000);
        flyWheel = new FlywheelMC(hardwareMap,"shooter",600000);

        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setBackwardsEncoder(true);

        flyWheel.setHistorySize(3);


        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setFlywheel(shooterDirection.OUT);
        flyWheel.setPowerManual(flywheelPower);
        flyWheel.setPowerScale(150/100000000.0);
        flyWheel.setLookAheadTime(.35);
        setLaunchServo(false);


    }

    public void tick () {
        long now = System.nanoTime();
        Long entry = null;
        for (Long when : toggleQueue) {
            if (now>when){
                entry = when;
            }
        }
        if (entry != null) {
            setLaunchServo(!launchServoState);
            toggleQueue.remove(entry);
        }
        flyWheel.setPowerAuto(flywheelPower);
    }

    public void clearQueue (){
        toggleQueue.clear();
    }

    public void addQueue (int whenMS) {
        toggleQueue.add(System.nanoTime()+(whenMS*1000000));
    }

    public void oldShoot() {
        setLaunchServo(true);
        opMode.sleep(200);
        setLaunchServo(false);
        opMode.sleep(200);

    }
    public void shoot () {
        clearQueue();
        setFlywheel(UGRobot.shooterDirection.OUT);
        setLaunchServo(true);
        addQueue(200);
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
            launchServoState = true;
        } else {
            launchServo.setPosition(0.55);
            launchServoState = false;
        }
    }

    public double getIdle() {
        return idle;
    }

    public void setIdle(double idle) {
        this.idle = idle;
    }

    public double getFlywheelPower() {
        return flywheelPower;
    }

    public void setFlywheelPower(double flywheelPower) {
        this.flywheelPower = flywheelPower;
    }

    public int getShooterEncoderPosition() {
        return flyWheel.motor.getCurrentPosition();
    }

    public void setFlywheel(UGRobot.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case OUT:
                flyWheel.setPowerAuto(flywheelPower);
                break;
            case STOP:
                flyWheel.setPowerManual(0);
                break;
        }
    }
}