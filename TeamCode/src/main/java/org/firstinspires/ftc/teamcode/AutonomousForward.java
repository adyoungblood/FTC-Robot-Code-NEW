package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutonomousForward extends OpMode {

    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;
    public double x;

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

        driveautonomous(0.4, 3);

    }

    public void driveautonomous (double power_variable, int time_variable) {
        motor_drive_right.setPower(power_variable);
        motor_drive_left.setPower(power_variable);

        try {
            TimeUnit.SECONDS.sleep(time_variable);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
}


