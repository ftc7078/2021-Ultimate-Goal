

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder Test", group="Linear Opmode")

public class EncoderTest extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor;
    HPMC shooter;
    long nextWake;
    long tickTime = 50;

    @Override
    public void runOpMode() {
        shooter = new HPMC(hardwareMap, "shooter", 2800);
         telemetry.addData("Status", "Initialized");
         waitForStart();
         runtime.reset();
         telemetry.clear();
         shooter.setPowerAuto(0.12);
         while (opModeIsActive()) {
            telemetry.addData("Time:", runtime.seconds());
            shooter.updateCurrentVelocity();
            
            shooter.autoAdjust();
            telemetry.addData("Velocity", shooter.getCurrentVelocity());
            telemetry.addData("RawPower", shooter.getPower());
            telemetry.update();
            tickSleep();
         }
    }
    
    public void tickSleep() {

        long now = System.nanoTime();
        nextWake = nextWake + tickTime * 1000000;
        if (nextWake < now) {
            nextWake = now + tickTime * 1000000;
            double msLate = (now - nextWake) / 1000000;
            if (msLate > 100) {
                System.out.println(String.format("Either this is the first tick or something is wrong: %.1f ms late", msLate));
                throw new RuntimeException(String.format("Can't keep up: tick took %.1f ms too long", msLate));
            }
            return;
        }
        long sleepTime = (int) Math.floor((nextWake - now) / 1000000);
        //System.out.println("Sleeping: " + sleepTime);
        sleep(sleepTime);

    }
}