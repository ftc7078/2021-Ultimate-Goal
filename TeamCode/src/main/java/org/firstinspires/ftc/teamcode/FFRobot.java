/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

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
    private Servo pickupDoor;
    private DcMotor pickup;
    public DcMotor arm;
    private DcMotor duckWheel;
    private int high = 1200;
    private int middle = 600;
    private int low = 500;
    private int pickUp = 000;
    private double armPower = 0.3;
    public final static double DOOR_DOWN = 0;
    public final static double DOOR_UP= 1;



    public enum armPosition {HIGH,MIDDLE,LOW,PICKUP}


    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private Servo foundationLeft;
    private Servo foundationRight;
    public ManipulatorDirection manipulatorState;
    public boolean manipulatorAutostop = false;
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
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.moveArm(armPosition.PICKUP);

        pickup.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveArm (armPosition targetPosition){
        if (targetPosition == armPosition.HIGH){
            arm.setTargetPosition(high);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            pickupDoor.setPosition(DOOR_UP);
        } else if (targetPosition == armPosition.MIDDLE){
            arm.setTargetPosition(middle);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            pickupDoor.setPosition(DOOR_UP);
        } else if (targetPosition == armPosition.LOW){
            arm.setTargetPosition(low);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            pickupDoor.setPosition(DOOR_UP);
        } else if (targetPosition == armPosition.PICKUP){
            arm.setTargetPosition(pickUp);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);
            pickupDoor.setPosition(DOOR_DOWN);
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
    public void setShippingElementPickupPosition(double position){
        shippingElementPickup.setPosition(position);
    }
}
