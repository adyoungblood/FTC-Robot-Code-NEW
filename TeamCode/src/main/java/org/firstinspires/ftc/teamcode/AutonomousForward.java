package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.R.attr.x;

@Autonomous
public class AutonomousForward extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;

    double instant;
    int x = 0;

    @Override
    public void init() {
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (x == 0) {
            motor_drive_right.setPower(0.75);
            motor_drive_left.setPower(0.75);


            instant = runtime.milliseconds();
            while (instant > runtime.milliseconds() - 4000) {
                telemetry.addData("Time Left", (4000 - (runtime.milliseconds() - instant)));
            }

            motor_drive_left.setPower(0);
            motor_drive_right.setPower(0);

            /*
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            motor_drive_left.setPower(0);
            motor_drive_right.setPower(0);
            */
            x++;
        } else {
            telemetry.addData("Status", "Autonomous done");
        }
    }

}