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

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
public class CameraVisionLeftRapid extends LinearOpMode
{

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

    double reductieMotor = 0.8;
    double vitezaSlider = 0.8;
    double vitezaMotor = 0.6;
    int nrConuri = 6;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int location1 = 0;
    int location2 = 1;
    int location3 = 2;
    int location = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "CameraRight"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == location1 || tag.id == location2 || tag.id == location3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    telemetry.addData("CAZUL: ", tagOfInterest.id+1);
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //actual code
        /****************
         ____ ___  ____
         / ___/ _ \|  _ \
         | |  | | | | | | |
         | |__| |_| | |_| |
         \____\___/|____/
         ****************/
        if(tagOfInterest == null || tagOfInterest.id == location1)
        {
            location = location1;
        }else if(tagOfInterest.id == location2)
        {
            location = location2;
        }else if(tagOfInterest.id == location3)
        {
            location = location3;
        }
        //incepem la 0
        //ridicam mecanismul
        ridicaBrat();

        //resetam slideru
        reset();

        //ridicam slideru la nivel
        urca(1150); //1150

        //mergem in stanga sa ne pozitionam la bat
        stanga(810, 0.4);
        stai(10);

        //lasa conul preloaded
        coboara(300); //850
        cleste.setPosition(0.2);
        stai(10);

        //merge stanga pt conuri
        stanga(1960, 0.4);


        //coboara slideru la primu con
        coboara(420); //430
        stai(100);
        cleste.setPosition(0.2);

        //se roteste 180 grade
        rotire_stanga(1400, 0.4);
        stai(100);

        //merge pana la stacku de conuri
        fata(1075 , vitezaMotor);
        stai(10);

        //prinde primu con #1
        cleste.setPosition(0.6);
        stai(100);
        spate(120, vitezaMotor);

        //ridica la nivel #1
        urca(900); //1330
        stai(10);

        //merge in spate spre junction mic #1
        spate(400, vitezaMotor);
        stai(10);

        //se intoarce 90 stanga #1
        rotire_stanga(690, 0.4);
        stai(10);

        //se pozitioneaza la junction #1
        fata(78, vitezaMotor);
        //stai(10);

        //lasa primul con #1
        coboara(300); //1030
        cleste.setPosition(0.2);
        stai(10);
        spate(100, vitezaMotor);

        //se intoarce 90 dreapta
        stai(50);
        rotire_dreapta(690, 0.4);
        stai(10);

        //pune clestele la nivel din nou #2
        coboara(700); //330
        stai(10);
        cleste.setPosition(0.2);

        //merge spre conuri din nou #2
        fata(510, vitezaMotor);
        stai(100);

        //prinde al doilea con #2
        cleste.setPosition(0.6);
        stai(100);
        spate(100, vitezaMotor);

        //ridica la nivel #2
        urca(880); //1210
        stai(10);

        //merge in spate spre junction mic #2
        spate(430, vitezaMotor);
        stai(10);

        //se intoarce 90 stanga #2
        rotire_stanga(690, vitezaMotor);
        stai(10);

        //se pozitioneaza la junction #2
        fata(50, vitezaMotor);
        //stai(10);

        //lasa conul
        coboara(300);
        cleste.setPosition(0.2);
        stai(10);
        spate(125, vitezaMotor);


        if(location == 0)
        {
            dreapta(575, 0.5);
           // spate(1100, 0.3);
        }
        if(location == 1)
        {
            stanga(575, 0.5);
        }
        if(location == 2)
        {
            stanga(1750, 0.5);
           // fata(1100, 0.3);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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
        cleste.setPosition(0.2);
        while (slider.isBusy()) {
            if (!touch.isPressed()) {
                slider.setPower(vitezaSlider);
                cleste.setPosition(0.7);
            }
            else {
                slider.setPower(0);
                break;
            }
        }
    }
    public void urca(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(vitezaSlider);
        }
    }
    public void coboara(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(-vitezaSlider);
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
    public void josN(int pos) {
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(-pos);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (slider.isBusy()) {
            slider.setPower(-vitezaSlider);
        }
    }
    public void ridicaBrat(){
        cleste.setPosition(0.5);
        intinzator.setPosition(0);
        exbazadr.setPower(1);
        exbazast.setPower(1);
        sleep(600);
        exsus.setPosition(0.7);
        sleep(2200);
        exbazadr.setPower(0);
        exbazast.setPower(0);
        sleep(300);
        intinzator.setPosition(0.15);
        cleste.setPosition(0.5);
        sleep(500);
    }
    public void rotire_dreapta(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(pos);
        fdr.setTargetPosition(-pos);
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
    public void rotire_stanga(int pos, double vitezaMotor)
    {
        fst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fst.setTargetPosition(-pos);
        fdr.setTargetPosition(pos);
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

    public void stai(int t)
    {
        fst.setPower(0);
        fdr.setPower(0);
        sst.setPower(0);
        sdr.setPower(0);
        sleep(t);
    }
}