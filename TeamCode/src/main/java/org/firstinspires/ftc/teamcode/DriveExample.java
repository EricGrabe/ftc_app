package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveExample", group = "Iterative OpMode" )

public class DriveExample extends OpMode {
    private DcMotor dcmotor_fr = null;
    private DcMotor dcmotor_fl = null;
    private DcMotor dcmotor_br = null;
    private DcMotor dcmotor_bl = null;

    double motorPower = 1;

@Override
public void init() {
    dcmotor_fr = hardwareMap.dcMotor.get("dcmotor_fr");  //controller 2 port 1
    dcmotor_fl = hardwareMap.dcMotor.get("dcmotor_fl");  //controller 1 port 1
    dcmotor_bl = hardwareMap.dcMotor.get("dcmotor_bl");  //controller 1 port 2
    dcmotor_br = hardwareMap.dcMotor.get("dcmotor_br");  //controller 2 port 2
}
@Override
public void init_loop() {
    dcmotor_fr.setPower(0);
    dcmotor_fl.setPower(0);
    dcmotor_br.setPower(0);
    dcmotor_br.setPower(0);
}
@Override
public void start() {
    //runtime.reset();
}
@Override
public void loop() {
    dcmotor_fl.setPower(gamepad1.left_stick_y * motorPower);
    dcmotor_fr.setPower(-gamepad1.right_stick_y * motorPower);
    dcmotor_bl.setPower(gamepad1.left_stick_y * motorPower);
    dcmotor_br.setPower(-gamepad1.right_stick_y * motorPower);
}
@Override
public void stop(){
    dcmotor_br.setPower(0);
    dcmotor_bl.setPower(0);
    dcmotor_fl.setPower(0);
    dcmotor_fr.setPower(0);

}

}


