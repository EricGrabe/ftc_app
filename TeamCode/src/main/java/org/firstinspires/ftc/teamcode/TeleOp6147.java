/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *tele-Op program 2016-2017
 */

@TeleOp(name="TeleOp6147", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public abstract class TeleOp6147 extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    // declare all DC motors and related variables
    private DcMotor dcmotor_fl = null;
    private DcMotor dcmotor_fr = null;
    private DcMotor dcmotor_bl = null;
    private DcMotor dcmotor_br = null;
    private DcMotor dcmotor_g = null;
    double motorPower = 1;


    //declare all servo motors and related varaibles
    Servo servoArm = null;

    double closed = 0.2;
    double open = 0.8;


    //declare all sensors and related variables
  // public UltrasonicSensor ultra = null;
    //HiTechnicNxtUltrasonicSensor ultrasonic;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        dcmotor_fl = hardwareMap.dcMotor.get("dcmotor_fl");
        dcmotor_fr = hardwareMap.dcMotor.get("dcmotor_fr");
        dcmotor_bl = hardwareMap.dcMotor.get("dcmotor_bl");
        dcmotor_br = hardwareMap.dcMotor.get("dcmotor_br");
        dcmotor_g = hardwareMap.dcMotor.get("dcmotor_g");
        //ultra = hardwareMap.ultrasonicSensor.get("ultra");

        servoArm = hardwareMap.servo.get("servoArm");


        // eg: Set the drive motor directions:
        //

        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "looping");
        dcmotor_fl.setPower(0);
        dcmotor_fr.setPower(0);
        dcmotor_bl.setPower(0);
        dcmotor_br.setPower(0);
        dcmotor_g.setPower(0);
        servoArm.setPosition(.6);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        dcmotor_fl.setPower(gamepad1.left_stick_y * motorPower);
        dcmotor_fr.setPower(-gamepad1.right_stick_y * motorPower);
        dcmotor_bl.setPower(gamepad1.left_stick_y * motorPower);
        dcmotor_br.setPower(-gamepad1.right_stick_y * motorPower);
        dcmotor_g.setPower(gamepad2.left_stick_y);  //scissor lift motors need to be opposite powers
        //dcmotor_gl.setPower(gamepad2.left_stick_y);  //scissor lift motors need to be opposite powers
        //dcmotor_gr.setPower(gamepad2.right_stick_y);   //scissor lift motors need to be opposite powers
        servoArm.setPosition(gamepad2.right_stick_y);


        if (gamepad1.right_bumper) {
            motorPower = 0.5;
        } else {
            motorPower = 1;
        }
       // telemetry.addData("MP", ultra.getUltrasonicLevel());


        if (gamepad1.a) {
            servoArm.setPosition(closed);
            //telemetry.addData("servoArm:", servoArm.getPosition());
        }
        if (gamepad1.b) {
            servoArm.setPosition(open);
            //telemetry.addData("servoArm:", servoArm.getPosition());


        }
    }

        /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop() {

        //set all motor values to zero
        dcmotor_fl.setPower(0.0);
        dcmotor_fr.setPower(0.0);
        dcmotor_bl.setPower(0.0);
        dcmotor_br.setPower(0.0);


        // servoArm_l.setPosition(min);
        // servoArm_r.setPosition(min);
    }
}








