package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OurColorSensorV2 {

    // Sensor and color-related fields
    private RevColorSensorV3 colorSensor;
    private final float[] hsvValues = new float[3];
    private View relativeLayout;
    private float baseGain;
    public float getBaseGain() { return colorSensor.getGain(); }

    /// initializes the color sensor
    public void initialize(HardwareMap hardwareMap, float gain) {

        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");

        colorSensor.enableLed(true);

        baseGain = gain;
        colorSensor.setGain(baseGain); // Set the gain
    }

    /** @return hsv values
     * @see NormalizedColorSensor
     * @see NormalizedRGBA
     * **/
    public float getRawHue() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        return hsvValues[0];
    }

    private double lighting;
    private double distance;

    public void collectData() {

        lighting = colorSensor.getLightDetected();
        distance = colorSensor.getDistance(DistanceUnit.MM);
    }

    public float getNormalizedHue() {
        return getRawHue(); //not programmed yet
    }

}