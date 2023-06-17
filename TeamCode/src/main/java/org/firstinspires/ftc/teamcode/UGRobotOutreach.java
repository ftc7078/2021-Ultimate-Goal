/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class UGRobotOutreach implements MecanumDrive.TickCallback {

    private Telemetry telemetry;
    private LinearOpMode opMode;



    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    FlywheelMC flyWheel = null;
    private Servo launchServo;
    public pickupDirection pickupState;
    public shooterDirection shooterState;
    private double flywheelPower = 0.72;
    private int upWobble = 1080;
    private int downWobble = 0;
    private int midWobble = 370;
    private int carryWobble = 610;
    private int lastPosition = 0;
    private double wobblePower = 0.75;
    private ArrayList<Long> toggleQueue = new ArrayList<Long>();
    private boolean launchServoState;
    int multishotDelay = 100;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {OUT, STOP}
    enum wobblePosition {UP,DOWN,MID,CARRY}
    //enum wobbleDirection {UP,DOWN,STOP}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"launchServo");

        pickupbottom = hardwareMap.get(DcMotor.class, "pickupBottom");
        pickuptop = hardwareMap.get(DcMotor.class, "pickupTop");

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



        telemetry.addData("last:", lastPosition);
        telemetry.addData("timea",opMode.getRuntime());

        opMode.sleep(50);
        telemetry.addData("timeb",opMode.getRuntime());
        telemetry.update();


    }
    public void setMotorDirections (MecanumDrive mecanumDrive) {
        mecanumDrive.setMotorDirections(FORWARD, REVERSE, FORWARD, REVERSE);

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

    public void tickCallback() {
        tick();
    }





    public void multiShoot (){
        clearQueue();
        setFlywheel(UGRobotOutreach.shooterDirection.OUT);
        setLaunchServo(true);
        for (int i=1;i<=3;i++) {
            addQueue(multishotDelay * i);
        }
        for (int i=5;i<=8;i++) {
            addQueue(multishotDelay * i);
        }

    }

    //public void addWobble(UGRobot.wobbleDirection direction) {
        //wobbleState = direction;
        //case UP:
    //}

    public void clearQueue (){
        toggleQueue.clear();
    }

    public boolean notDoneShooting() {
        return !toggleQueue.isEmpty();
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
        setFlywheel(UGRobotOutreach.shooterDirection.OUT);
        setLaunchServo(true);
        addQueue(200);
    }

    public double findShooterSpeed () {
        flyWheel.updateCurrentVelocity();
        return (flyWheel.getCurrentVelocity());
    }

    public void setPickup(UGRobotOutreach.pickupDirection direction) {
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

    public double getFlywheelPower() {
        return flywheelPower;
    }

    public void setFlywheelPower(double flywheelPower) {
        this.flywheelPower = flywheelPower;
    }

    public int getShooterEncoderPosition() {
        return flyWheel.motor.getCurrentPosition();
    }

    public void setFlywheel(UGRobotOutreach.shooterDirection direction) {
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
