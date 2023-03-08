/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class Showoff extends LinearOpMode
{
    Servo exsus, intinzator, cleste;
    CRServo exbazast, exbazadr;

    double vitezaSlider = 0.8;

    @Override
    public void runOpMode()
    {

        //servo uri
        exbazast = hardwareMap.crservo.get("exbazast"); // in pinul 0
        exbazast.setDirection(CRServo.Direction.REVERSE);
        exbazadr = hardwareMap.crservo.get("exbazadr"); // in pinul 1
        //exbaza2.setDirection(Servo.Direction.REVERSE);
        exsus = hardwareMap.servo.get("exsus");
        intinzator = hardwareMap.servo.get("intinzator");
        //intinzator.setDirection(Servo.Direction.REVERSE);
        cleste = hardwareMap.servo.get("cleste");
       while(opModeIsActive()) {
           ridicaBrat();
           coboaraBrat();
       }
    }

    public void ridicaBrat(){
        exbazadr.setPower(1);
        exbazast.setPower(1);
        sleep(600);
        exsus.setPosition(0.7);
        sleep(2200);
        exbazadr.setPower(0);
        exbazast.setPower(0);
        sleep(300);
        sleep(500);
    }

    public void coboaraBrat(){
        exbazadr.setPower(-1);
        exbazast.setPower(-1);
        sleep(600);
        exsus.setPosition(0);
        sleep(2200);
        exbazadr.setPower(0);
        exbazast.setPower(0);
        sleep(300);
        sleep(500);
    }
}