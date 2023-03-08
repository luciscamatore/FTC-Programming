package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Disabled
@TeleOp
public class TeleOpTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "model_20220216_195212.tflite";
    private static final String[] LABELS = {
            "TSE"
    };
    private static final String VUFORIA_KEY =
            "AXea9Af/////AAABmXkJssyFt0iWhZPf0JjyRbIl6AI1zZwBsw3gB+ULAv6vvt7AmINgZcObH0Os3hwhF6vuHUjPnqhKmgvwuQnKhxfUQ4GRmNAqeDxbZBaDKw/P10qiZkrT8R34eAMVZkgugm55RZCN1R/gLwtAQnb+/B6iYkmeItUnvIf74mB6fMwwCM7eX/H1L8ke3w2prQ2ZiLCoRARM+ZRKdPBE1x/u4EyzJ+cFJ/eg1QOFFZOnkaOewZbesLU++qQvHO4/tO2VzQrbh86tVGgCsJqzlyjMbIL9BXNSUcxXcpa6CFixaGOS5g65eScfchxKaUOUhL9dj3HJO3/l2UPQJujFC3BuOaq8qNJuCtSJ/5sv/35VjOKH";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    int p;

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.3, 16.0/9.0);
        }


        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("pozitie: ", pozitie());
            telemetry.update();

        }
    }
    public int pozitie()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions) {
                    //CONDITII
                    if(recognition.getLeft() < 200) {
                        p=1;
                    }
                    else if(recognition.getLeft() > 200 && recognition.getLeft() < 400)
                    {
                        p=2;
                    }else if(recognition.getLeft() > 400)
                        p= 3;

                }
                telemetry.update();
            }
        }
        return p;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}