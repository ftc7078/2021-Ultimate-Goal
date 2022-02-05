/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FFRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;
    private Servo shippingElementPickup;
    Servo pickupDoor;
    DcMotor pickup;
    public DcMotor arm;
    private DcMotor duckWheel;
    private int high = 1500;
    private int carry = 600;
    private int pickUp = 000;
    private double armPower = 0.7;
    public final static double DOOR_DOWN = 0;
    public final static double DOOR_UP= 1;



    public enum armPosition {HIGH,CARRY,PICKUP}
    public enum doorPosition {PICKUP,CARRY,DUMP}



    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private Servo foundationLeft;
    private Servo foundationRight;
    public ManipulatorDirection manipulatorState;
    public boolean manipulatorAutostop = false;
    public int dropped=0;
    public doorPosition currentDoorPosition=null;
    public armPosition currentArmPosition=null;
    //bellow is old
    Servo capstoneArm;
    Servo capstoneDrop;
    Servo inRamp;
    DigitalChannel digitalTouch;
    enum ManipulatorDirection { IN, OUT, STOP}

    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        shippingElementPickup = hardwareMap.get(Servo.class, "shipping_element_pickup");
        pickupDoor = hardwareMap.get(Servo.class, "pickup_door");
        pickup = hardwareMap.get(DcMotor.class, "pickup");
        arm = hardwareMap.get(DcMotor.class, "arm");
        duckWheel = hardwareMap.get(DcMotor.class, "duck_wheel");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        Log.d("FFRobot", "init: "+pickupDoor.getPosition());



        this.arm.setPower(-0.1);
        int lastPosition = arm.getCurrentPosition();
        int startPosition = arm.getCurrentPosition();


        opMode.sleep(50);
        while (arm.getCurrentPosition() != lastPosition) {
            lastPosition=arm.getCurrentPosition();
            opMode.sleep(50);
        }
        dropped=arm.getCurrentPosition() - startPosition;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.setDoorPosition(doorPosition.CARRY);
        this.moveArm(armPosition.PICKUP);

        pickup.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveArm (armPosition targetPosition){
        currentArmPosition=targetPosition;
        if (targetPosition == armPosition.HIGH){
            arm.setTargetPosition(high);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);

        } else if (targetPosition == armPosition.CARRY){
            arm.setTargetPosition(carry);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);

        } else if (targetPosition == armPosition.PICKUP){
            arm.setTargetPosition(pickUp);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower/2);
        }
    }
    public void pickup(boolean on) {
        if (on) {
            pickup.setPower(0.7);
        } else {
            pickup.setPower(0);
        }
    }
    public void setDuckWheel(double power){
        duckWheel.setPower(power);
    }
    public void setDoorPosition(doorPosition position) {
        currentDoorPosition=position;
        if (currentArmPosition == armPosition.PICKUP) {
            position = doorPosition.PICKUP;
        }
        if (position == doorPosition.CARRY) {
            pickupDoor.setPosition(0.3);
        }
        if (position == doorPosition.DUMP) {
            pickupDoor.setPosition(1);
        }
        if (position == doorPosition.PICKUP) {
            pickupDoor.setPosition(0);
        }
    }
    public void setShippingElementPickupPosition(double position){
        shippingElementPickup.setPosition(position);
    }
}
