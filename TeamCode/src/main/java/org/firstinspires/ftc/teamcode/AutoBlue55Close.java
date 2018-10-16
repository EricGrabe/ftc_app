/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoBlue55Close", group="Linear Opmode")
//@Disabled
public class AutoBlue55Close extends LinearOpMode {

    // Declare OpMode members.
    // declare all DC motors and related variables
    private DcMotor dcmotor_fl = null;     //S0 dcmotorcontroller1 port 1
    private DcMotor dcmotor_fr = null;     //S1 dcmotorcontroller2 port 1
    private DcMotor dcmotor_bl = null;     //S0 dcmotorcontroller1 port 2
    private DcMotor dcmotor_br = null;     //S1 dcmotorcontroller2 port 2
    private DcMotor dcmotor_lift = null;   //S2 dcmotorcontroller3 port 1
    private DcMotor dcmotor_arm = null;    //S2 dcmotorcontroller3 port 2


    double motorPower = 1;


    //declare all servo motors and related variables
    private Servo servoArmL = null;
    private Servo servoArmR = null;
    double servoValue = 1;
    double min = 0.2;
    double max = 0.8;
    double open = 1.0;
    double closed = .55;
    double grip = 0.50;
    double drop = 0.50;


    //declare all sensors and related variables
    ColorSensor color_sensor;
    int color_sensor1 = 4500;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        dcmotor_fl = hardwareMap.dcMotor.get("dcmotor_fl");
        dcmotor_fr = hardwareMap.dcMotor.get("dcmotor_fr");
        dcmotor_bl = hardwareMap.dcMotor.get("dcmotor_bl");
        dcmotor_br = hardwareMap.dcMotor.get("dcmotor_br");
        dcmotor_lift = hardwareMap.dcMotor.get("dcmotor_lift");
        dcmotor_arm = hardwareMap.dcMotor.get("dcmotor_arm");
        servoArmL = hardwareMap.servo.get("servoArmL");
        servoArmR = hardwareMap.servo.get("servoArmR");
        color_sensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Ready", 0);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    public void stopBot(long time) throws InterruptedException{

        //set all motor values to zero
        dcmotor_fl.setPower(0.0);
        dcmotor_fr.setPower(0.0);
        dcmotor_bl.setPower(0.0);
        dcmotor_br.setPower(0.0);
        dcmotor_lift.setPower(0.0);
        dcmotor_arm.setPower(0.0);
        Thread.sleep(time);
        //set all servo positions to min
        //servoArm.setPosition(min);
        // jewelServo.setPosition(.3);
    }

    void driveForward(double power, long time) throws InterruptedException {
        dcmotor_fl.setPower(-power);
        dcmotor_fr.setPower(power);
        dcmotor_bl.setPower(-power);
        dcmotor_br.setPower(power);
        Thread.sleep(time);
        stopBot(100);
    }

    void driveBackward(double power, long time) throws InterruptedException {
        dcmotor_fl.setPower(power);
        dcmotor_fr.setPower(-power);
        dcmotor_bl.setPower(power);
        dcmotor_br.setPower(-power);
        Thread.sleep(time);
        stopBot(100);
    }

    void TurnCounterclockwise(double power, long time) throws InterruptedException {
        dcmotor_fl.setPower(power);
        dcmotor_fr.setPower(power);
        dcmotor_bl.setPower(power);
        dcmotor_br.setPower(power);
        Thread.sleep(time);
        stopBot(100);
    }

    void TurnClockwise(double power, long time) throws InterruptedException {
        dcmotor_fl.setPower(-power);
        dcmotor_fr.setPower(-power);
        dcmotor_bl.setPower(-power);
        dcmotor_br.setPower(-power);
        Thread.sleep(time);
        stopBot(100);
    }

    void gripGlyph() throws InterruptedException{
        servoArmL.setPosition(grip);
        servoArmR.setPosition(1-grip);
    }

    void dropGlyph() throws InterruptedException {
        servoArmL.setPosition(drop);
        servoArmR.setPosition(grip);
    }

    void openGate() throws InterruptedException {
        servoArmL.setPosition(open);
        servoArmR.setPosition(closed);
    }

    void closeGate () throws InterruptedException {
        servoArmR.setPosition(open);
        Thread.sleep(250);
        servoArmL.setPosition(closed);
    }

    public void hitJewel() throws InterruptedException {
        telemetry.addData("Red", color_sensor.red());
        telemetry.addData("Green", color_sensor.green());
        telemetry.addData("Blue", color_sensor.blue());
        if ((color_sensor.red() > color_sensor.blue())) {
            } else if ((color_sensor.blue() > color_sensor.red())) {
            }
        stopBot(100);
    }

    public void liftGate(double power, long time) throws InterruptedException {
        dcmotor_lift.setPower(-power);
        Thread.sleep(time);
        stopBot(100);
    }

    public void extendSlide(double power, long time) throws InterruptedException {
        while(color_sensor.red() + color_sensor.blue() < 40)
            dcmotor_arm.setPower(power);
        //Thread.sleep(time);
        stopBot(time);
    }

    public void retractSlide(double power, long time) throws InterruptedException {
        dcmotor_arm.setPower(-power);
        Thread.sleep(time);
        stopBot(100);
    }
}