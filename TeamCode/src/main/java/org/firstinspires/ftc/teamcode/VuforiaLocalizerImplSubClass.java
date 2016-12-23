package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.vuforia.CameraDevice;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.State;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl;


//import java.text.SimpleDateFormat;
//import java.util.Date;

public class VuforiaLocalizerImplSubClass extends VuforiaLocalizerImpl {

    public Image rgb;
    private ElapsedTime runtime = new ElapsedTime();

    public class CloseableFrame extends Frame {
        public CloseableFrame(Frame other) {
            super(other);
        }

        public void close() {
            super.delete();
        }
    }

    public class VuforiaCallbackSubClass extends VuforiaLocalizerImpl.VuforiaCallback {

        @Override
        public synchronized void Vuforia_onUpdate(State state) {
            super.Vuforia_onUpdate(state);

            CloseableFrame frame = new CloseableFrame(state.getFrame());

            long numImages = frame.getNumImages();
            Image rgb = null;

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }
            }
        }
    }
    public  VuforiaLocalizerImplSubClass(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
        stopAR();
        clearGLSurface();

        this.vuforiaCallback = new VuforiaCallbackSubClass();
        startAR();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public void clearGLSurface() {
        if (this.glSurfaceParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override public void run() {
                    glSurfaceParent.removeAllViews();
                    glSurfaceParent.getOverlay().clear();
                    glSurface = null;
                }
            });
        }
    }
}
