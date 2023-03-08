package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

@Disabled
@Autonomous

public class AutonomieEncoderSimplu extends LinearOpMode {

    //motoare
    DcMotor fst;
    DcMotor sdr;
    DcMotor fdr;
    DcMotor sst;

    //SliderThread sliding = new SliderThread();

    DcMotor slider;
    Servo exsus, intinzator, cleste;
    CRServo exbazast, exbazadr;


    TouchSensor touch;

    double vitezaSlider = 0.8;

    public void runOpMode() throws InterruptedException{

        //motoare pentru sasiu
        fst = hardwareMap.dcMotor.get("fst");
        fst.setDirection(DcMotorSimple.Direction.REVERSE);
        fst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setDirection(DcMotorSimple.Direction.REVERSE);
        sst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider = hardwareMap.dcMotor.get("slider");
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        exbazast = hardwareMap.crservo.get("exbazast"); // in pinul 0
        exbazast.setDirection(CRServo.Direction.REVERSE);
        exbazadr = hardwareMap.crservo.get("exbazadr"); // in pinul 1
        exsus = hardwareMap.servo.get("exsus");
        intinzator = hardwareMap.servo.get("intinzator");
        cleste = hardwareMap.servo.get("cleste");

        touch = hardwareMap.touchSensor.get("touch");

        waitForStart();





    }
    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/
    public void reset() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-10000);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.isBusy()) {
            if (!touch.isPressed()) {
                slider.setPower(vitezaSlider);
            }
            else {
                slider.setPower(0);
                break;
            }
        }
    }
    public void jos(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(vitezaSlider);
        }
    }
    public void fata(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(pos);
        fdr.setTargetPosition(pos);
        sst.setTargetPosition(pos);
        sdr.setTargetPosition(pos);
        fst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (sdr.isBusy()) {
            fst.setPower(vitezaMotor);
            fdr.setPower(vitezaMotor);
            sst.setPower(vitezaMotor);
            sdr.setPower(vitezaMotor);
            telemetry.addData("pos", sdr.getCurrentPosition());
            telemetry.update();
        }
    }
    public void spate(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(-pos);
        fdr.setTargetPosition(-pos);
        sst.setTargetPosition(-pos);
        sdr.setTargetPosition(-pos);
        fst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (sdr.isBusy()) {
            fst.setPower(vitezaMotor);
            fdr.setPower(vitezaMotor);
            sst.setPower(vitezaMotor);
            sdr.setPower(vitezaMotor);
            telemetry.addData("pos", sdr.getCurrentPosition());
            telemetry.update();
        }
    }
    public void stanga(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(-pos);
        fdr.setTargetPosition(pos);
        sst.setTargetPosition(pos);
        sdr.setTargetPosition(-pos);
        fst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (sdr.isBusy()) {
            fst.setPower(vitezaMotor);
            fdr.setPower(vitezaMotor);
            sst.setPower(vitezaMotor);
            sdr.setPower(vitezaMotor);
            telemetry.addData("pos", sdr.getCurrentPosition());
            telemetry.update();
        }
    }
    public void dreapta(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(pos);
        fdr.setTargetPosition(-pos);
        sst.setTargetPosition(-pos);
        sdr.setTargetPosition(pos);
        fst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (sdr.isBusy()) {
            fst.setPower(vitezaMotor);
            fdr.setPower(vitezaMotor);
            sst.setPower(vitezaMotor);
            sdr.setPower(vitezaMotor);
            telemetry.addData("pos", sdr.getCurrentPosition());
            telemetry.update();
        }
    }

}
