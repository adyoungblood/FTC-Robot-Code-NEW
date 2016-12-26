package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name="Autonomous03", group="Iterative Opmode")  // @TeleOp(...) is the other common choice

enum State {
    INITIALIZE, MOVE, CHECK, STOP
}

public class Autonomous03 extends OpMode {
    State state_s;

    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;
    public DcMotor intake_motor_2;

    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_back;
    //    public double power_RT;
//    public double power_LT;
    public double power_level;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_shooter;


    @Override
    public void init() {
        state_s = State.INITIALIZE;
    }

    @Override
    public void start() {
        state_s = State.INITIALIZE;
    }

    @Override
    public void loop() {
        switch (state_s) {
            case INITIALIZE:
                state_s = State.MOVE;
                resetStartTime();
                break;
            case MOVE:
                drive(1, 1, 1);
                state_s = State.CHECK;
                break;
            case CHECK:
                if (this.getRuntime() > 1000) { state_s = State.STOP; };
                break;
            case STOP:
                this.stop();
                break;
            default:
                state_s = State.STOP;
                break;
        }
    }


    public void drive(int left, int right, int time) {
        motor_drive_left.setPower(left);
        motor_drive_right.setPower(right);
        try {
            Thread.sleep(time * 1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motor_drive_left.setPower(0);
        motor_drive_right.setPower(0);
    }
}