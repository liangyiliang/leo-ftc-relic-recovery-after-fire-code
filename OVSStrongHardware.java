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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;


public class OVSStrongHardware
{
    public enum TurnDirection{LEFT, RIGHT, NONE};
    /* Public OpMode members. */
    public DcMotor lfDrive       = null;
    public DcMotor lbDrive       = null;
    public DcMotor rfDrive       = null;
    public DcMotor rbDrive       = null;
    public Servo   lfServo       = null;
    public Servo   rfServo       = null;
    public DcMotor parallex      = null;
    public DcMotor relicExtender = null;
    public Servo   wrist         = null;
    public Servo   hand          = null;
    public Servo   jewelKicker   = null;
    public ColorSensor color = null;
    public ColorSensor lColor = null;
    public ColorSensor rColor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OVSStrongHardware(){}

    private double remainPower(double p){
        return 1 - Math.abs(p);
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        lfDrive = hwMap.get(DcMotor.class, "lfdrive");
        lbDrive = hwMap.get(DcMotor.class, "lbdrive");
        rfDrive = hwMap.get(DcMotor.class, "rfdrive");
        rbDrive = hwMap.get(DcMotor.class, "rbdrive");
        lfDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        lbDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rfDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rbDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);

        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfServo = hwMap.get(Servo.class, "lfservo");
        lfServo.setPosition(0.5);
        rfServo = hwMap.get(Servo.class, "rfservo");
        lfServo.setPosition(0.5);

        parallex = hwMap.get(DcMotor.class, "parallex");
        parallex.setDirection(DcMotorSimple.Direction.REVERSE);
        relicExtender = hwMap.get(DcMotor.class, "relicext");

        wrist = hwMap.get(Servo.class, "wrist");
        wrist.setPosition(0.5);
        hand  = hwMap.get(Servo.class, "hand");
        hand.setPosition(0);

        jewelKicker = hwMap.get(Servo.class, "auto");
        jewelKicker.setPosition(1);

        color = hwMap.get(ColorSensor.class, "color");
        color.enableLed(true);

        lColor = hwMap.get(ColorSensor.class, "lcolor");
        lColor.enableLed(true);
        rColor = hwMap.get(ColorSensor.class, "rcolor");
        rColor.enableLed(true);
    }

    public void move(double theta, double magnitude, double turn, TurnDirection turnDir){
        double r = magnitude;
        double robotAngle = theta -  Math.PI /4;
        double triggerMag = turn;
        double v1 = r * Math.cos(robotAngle)/*+ rightX*/;
        double v2 = r * Math.sin(robotAngle)/*- rightX*/;
        double v3 = r * Math.sin(robotAngle)/*+ rightX*/;
        double v4 = r * Math.cos(robotAngle)/*- rightX*/;
        double turnPow = Math.abs(triggerMag) * remainPower(v1);
        if(turnDir == TurnDirection.RIGHT) {
            v1 += turnPow;
            v2 -= turnPow;
            v3 += turnPow;
            v4 -= turnPow;
        } else {
            v1 -= turnPow;
            v2 += turnPow;
            v3 -= turnPow;
            v4 += turnPow;
        }
        lfDrive.setPower(v1);
        rfDrive.setPower(v2);
        lbDrive.setPower(v3);
        rbDrive.setPower(v4);
    }


    public void shift(float power){
        float left_front_power  =  power;
        float left_back_power   = -power;
        float right_front_power = -power;
        float right_back_power  =  power;
        lfDrive.setPower(left_front_power);
        lbDrive.setPower(left_back_power);
        rfDrive.setPower(right_front_power);
        rbDrive.setPower(right_back_power);
    }

    public void lowerJewelKicker(){
        jewelKicker.setPosition(0);
    }
    public void retractJewelKicker(){
        jewelKicker.setPosition(1);
    }
    public void stop(){
        lfDrive.setPower(0);
        rfDrive.setPower(0);
        lbDrive.setPower(0);
        rbDrive.setPower(0);
    }
}