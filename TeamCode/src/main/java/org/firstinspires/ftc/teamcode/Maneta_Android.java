package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.threadopmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//Extend ThreadOpMode rather than OpMode
public class Maneta_Android extends ThreadOpMode {

    //Define global variables
    //motoare
    DcMotor fst;
    DcMotor sdr;
    DcMotor fdr;
    DcMotor sst;
    //SliderThread sliding = new SliderThread();

    DcMotor slider;
    Servo baza, arm, cleste;

    @Override
    public void mainInit() {
        telemetry.addData("bRUATTTTT "," CULCAT");
        telemetry.update();

        //motoare pentru sasiu
        fst = hardwareMap.dcMotor.get("fst");
        fst.setDirection(DcMotorSimple.Direction.REVERSE);
        fst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setDirection(DcMotorSimple.Direction.REVERSE);
        sst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SIDER
        slider = hardwareMap.dcMotor.get("slider");
        //slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //OTHER
        cleste = hardwareMap.servo.get("cleste");
        cleste.setDirection(Servo.Direction.REVERSE);
        baza = hardwareMap.servo.get("baza");
        arm = hardwareMap.servo.get("arm");
        //arm.setDirection(Servo.Direction.REVERSE);

        //Below is a new thread
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //The loop method should contain what to constantly run in the thread
                //For instance, this drives a single DcMotor
                slider.setPower(gamepad1.left_stick_y);
            }
        }));
    }

    @Override
    public void mainLoop() {
        //Anything you want to constantly run in the MAIN thread goes here
        sst.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        sdr.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
        fst.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        fdr.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
    }

}
