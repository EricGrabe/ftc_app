package org.firstinspires.ftc.teamcode;/*
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 */

@TeleOp(name="TeleOp20182019", group="Iterative Opmode")
//@Disabled
public class TeleOp20182019 extends OpMode {
    /* Declare OpMode members. */
    //robot.init(HardwareMap);
    private ElapsedTime runtime = new ElapsedTime();


    // declare all DC motors and related variables
    private DcMotor dcmotor_fl = null;
    private DcMotor dcmotor_fr = null;
    private DcMotor dcmotor_bl = null;
    private DcMotor dcmotor_br = null;
    private DcMotor dcmotor_lift = null;
    private DcMotor dcmotor_arm = null;

    double motorPower = 1;
    boolean safe = false;



    //declare all servo motors and related variables
    private Servo servoArmL = null;
    private Servo servoArmR = null;
    double open = 1;
    double closed = .85;

    //declare all sensors and related variables
      //ColorSensor color_sensor;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dcmotor_fl=hardwareMap.dcMotor.get("dcmotor_fl");        //S0 dcmotorcontroller1 port 1
        dcmotor_fr=hardwareMap.dcMotor.get("dcmotor_fr");        //S1 dcmotorcontroller2 port 1
        dcmotor_bl=hardwareMap.dcMotor.get("dcmotor_bl");        //S0 dcmotorcontroller1 port 2
        dcmotor_br=hardwareMap.dcMotor.get("dcmotor_br");        //S1 dcmotorcontroller2 port 2
        dcmotor_lift = hardwareMap.dcMotor.get("dcmotor_lift");  //S2 dcmotorcontroller3 port 1
        dcmotor_arm = hardwareMap.dcMotor.get("dcmotor_arm");    //S2 dcmotorcontroller3 port 2
        servoArmL = hardwareMap.servo.get("servoArmL");
        servoArmR= hardwareMap.servo.get("servoArmR");
        //color_sensor = hardwareMap.colorSensor.get("color_sensor");

        telemetry.addData("Ready", 0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //servoArm.setPosition(min);
        dcmotor_fl.setPower(0);
        dcmotor_fr.setPower(0);
        dcmotor_bl.setPower(0);
        dcmotor_br.setPower(0);
        dcmotor_lift.setPower(0);
        dcmotor_arm.setPower(0);
        telemetry.addData("Looping", 0);
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

        //lift gate with gamepad 2 left stick
        if(safe) {
            dcmotor_lift.setPower(gamepad2.left_stick_y);
        }
        else {
            dcmotor_lift.setPower(0);
        }

        dcmotor_arm.setPower(gamepad2.right_stick_x*0.5);
        /*
        //slows the robot down when "a" is pressed
        if(gamepad1.right_bumper) {
            motorPower = 0.5;
        } else {
            motorPower=1;
        }
        */

        //closes the gate if a is pressed on gamepad 1
        if(gamepad1.a) {
            servoArmL.setPosition(open);
            servoArmR.setPosition(1.0-open);
            safe=false;
        }

        //opens the gate if b is pressed on gamepad 1
        if(gamepad1.b) {
            servoArmL.setPosition(closed);
            servoArmR.setPosition(1-closed);
            safe=true;
        }



       //turn counter clockwise
        if(gamepad1.left_trigger>=0.1) {
            try {
                TurnCounterclockwise(gamepad1.left_trigger);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


        //turn clockwise
        else if(gamepad1.right_trigger>=0.1){
            try {
                TurnClockwise(gamepad1.right_trigger);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //full forward
        else if (gamepad1.left_stick_y>0.1 && gamepad1.right_bumper){
            dcmotor_bl.setPower(gamepad1.left_stick_y);
            dcmotor_fl.setPower(gamepad1.left_stick_y);
            dcmotor_br.setPower(-gamepad1.left_stick_y);
            dcmotor_fr.setPower(-gamepad1.left_stick_y);
        }

        //full backward
        else if(gamepad1.left_stick_y<-0.1 && gamepad1.right_bumper){
            dcmotor_bl.setPower(gamepad1.left_stick_y);
            dcmotor_fl.setPower(gamepad1.left_stick_y);
            dcmotor_br.setPower(-gamepad1.left_stick_y);
            dcmotor_fr.setPower(-gamepad1.left_stick_y);
        }

        //regular tank control
        else{
            dcmotor_fl.setPower(gamepad1.left_stick_y * motorPower);
            dcmotor_fr.setPower(-gamepad1.right_stick_y * motorPower);
            dcmotor_bl.setPower(gamepad1.left_stick_y * motorPower);
            dcmotor_br.setPower(-gamepad1.right_stick_y * motorPower);
        }
        //telemetry.addData("servoArm position", servoArmL.getPosition());
        //telemetry.addData("light value", sensor_light.getLightDetected());
        /*
        if(gamepad2.a){
            servoArmR.setPosition(1-open);
        }

        if(gamepad2.b) {
            servoArmL.setPosition(open);
        }
        */
        //telemetry.addData("red", color_sensor.red());
        //telemetry.addData("green", color_sensor.green());
        //telemetry.addData("blue", color_sensor.blue());
    }

    @Override
    public void stop() {

        //set all motor values to zero
        dcmotor_fl.setPower(0.0);
        dcmotor_fr.setPower(0.0);
        dcmotor_bl.setPower(0.0);
        dcmotor_br.setPower(0.0);
        dcmotor_lift.setPower(0.0);

    }

    void TurnCounterclockwise(double power) throws InterruptedException {

        dcmotor_fl.setPower(power);
        dcmotor_fr.setPower(power);
        dcmotor_bl.setPower(power);
        dcmotor_br.setPower(power);
        //dcmotor_lift.setPower(0);
        //Thread.sleep(time);
      //  stopRobot();
        //telemetry.addData("Time:", runtime.seconds());


    }

    void TurnClockwise(double power) throws InterruptedException {

        dcmotor_fl.setPower(-power);
        dcmotor_fr.setPower(-power);
        dcmotor_bl.setPower(-power);
        dcmotor_br.setPower(-power);
        //dcmotor_lift.setPower(0);
        //stopRobot();
        //telemetry.addData("Time:", runtime.seconds());


    }


    void stopRobot() {
        dcmotor_fl.setPower(0);
        dcmotor_fr.setPower(0);
        dcmotor_bl.setPower(0);
        dcmotor_br.setPower(0);
        dcmotor_lift.setPower(0);
    }

}


