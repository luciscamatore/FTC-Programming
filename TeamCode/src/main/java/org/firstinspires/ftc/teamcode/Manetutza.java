package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.threadopmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;

@TeleOp


public class Manetutza extends ThreadOpMode {

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

    double reductieMotor = 1;
    double vitezaSlider = 0.8;
    int nrConuri = 6;
    boolean resetare = false;
    boolean stack = false;

    @Override
    public void mainInit() {

        telemetry.addData("bRUATTTTT ", " CULCAT");
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

        //sider
        slider = hardwareMap.dcMotor.get("slider");
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servo uri
        exbazast = hardwareMap.crservo.get("exbazast"); // in pinul 0
        exbazast.setDirection(CRServo.Direction.REVERSE);
        exbazadr = hardwareMap.crservo.get("exbazadr"); // in pinul 1
        //exbaza2.setDirection(Servo.Direction.REVERSE);
        exsus = hardwareMap.servo.get("exsus");
        intinzator = hardwareMap.servo.get("intinzator");
        //intinzator.setDirection(Servo.Direction.REVERSE);
        cleste = hardwareMap.servo.get("cleste");

        //senzori
        touch = hardwareMap.touchSensor.get("touch");

        //thread slider
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //MOTOR SLIDER
                if (gamepad2.y && !resetare) {
                    resetare = true;
                    if(!stack)
                    {
                        actionareSlider(2700);
                    }else if(stack){
                        actionareSlider(2700-slider.getCurrentPosition());
                        stack = false;
                    }
                }
                if (gamepad2.x && !resetare) {
                    resetare = true;
                    if(!stack){
                        actionareSlider(1900);
                    }else if(stack){
                        actionareSlider(1900-slider.getCurrentPosition());
                        stack = false;
                    }
                }
                if (gamepad2.a && !resetare) {
                    resetare = true;
                    if(!stack)
                    {
                        actionareSlider(1150);
                    }else if(stack)
                    {
                        actionareSlider(1150 - slider.getCurrentPosition());
                        stack = false;
                    }
                }
                if (gamepad2.b) {
                    reset();
                    resetare = false;
                }
                conuri();
                if(gamepad2.dpad_right)
                {
                    resetEncoder();
                    resetare = false;
                }
            }
        }));

        //thread sasiu
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void loop() {
                //SASIU
                sst.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * reductieMotor);
                sdr.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * reductieMotor);
                fst.setPower(-(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * reductieMotor);
                fdr.setPower(-(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * reductieMotor);
            }
        }));
    }

    @Override
    public void mainLoop() {

        telemetry.addLine("Ernest: DEMASCAREA!");
        telemetry.update();

        intinzator.setPosition(0.15);
        //exsus.setPosition(0.5);
        exbazadr.setPower(0);
        exbazast.setPower(0);

        if (gamepad1.right_bumper) {
            cleste.setPosition(0.7);
        }
        if (gamepad1.left_bumper) {
            cleste.setPosition(0.2);
        }
    }

    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/

    public void actionareSlider(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            if(!gamepad2.right_bumper)
                slider.setPower(vitezaSlider);
            else if(gamepad2.right_bumper){
                slider.setPower(0);
                break;
            }
        }
    }

    public void reset() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-10000);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.isBusy()) {
            if (!touch.isPressed() && !gamepad2.right_bumper) {
                slider.setPower(vitezaSlider);
            }
            else if(touch.isPressed() || gamepad2.right_bumper){
                slider.setPower(0);
                resetare = true;
                break;
            }
        }
    }
    public void resetEncoder(){
        int pozitie  = -slider.getCurrentPosition();
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(pozitie);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.isBusy()) {
            slider.setPower(vitezaSlider);
        }
    }
    public void resetNumaratoare() {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-10000);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.isBusy()) {
            if (!touch.isPressed()  && !gamepad2.right_bumper) {
                slider.setPower(vitezaSlider);
            }
            else if(touch.isPressed() || gamepad2.right_bumper){
                slider.setPower(0);
                nrConuri--;
                break;
            }
        }
    }
    public void conuri() {
        int y = 66;
        if(gamepad2.dpad_up && nrConuri != 0)
        {
            resetNumaratoare();
            actionareSlider(y * nrConuri);
            stack = true;
            telemetry.addData("nrConuri ",nrConuri);
            telemetry.update();
        }
    }

    }

    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/

