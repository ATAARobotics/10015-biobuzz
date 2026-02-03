package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ConfigureColorRangefinder extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx operator = new GamepadEx(gamepad2);

        waitForStart();
        String mode = null;
        while (opModeIsActive() && mode == null) {
            operator.readButtons();
            if (operator.wasJustPressed(GamepadKeys.Button.A)) {
                mode = "telemetry";
            } else if (operator.wasJustPressed(GamepadKeys.Button.X)) {
                mode = "program";
            } else if (operator.wasJustPressed(GamepadKeys.Button.B)) {
                mode = "test";
            }
            telemetry.addData("A", "telemetry");
            telemetry.addData("X", "program");
            telemetry.addData("B", "test");
            telemetry.update();
        }

        // only some of these get set up depending what "mode" we're in
        RevColorSensorV3 sensor0 = null;
        RevColorSensorV3 sensor1 = null;
        ColorRangefinder crf0 = null;
        ColorRangefinder crf1 = null;
        DigitalChannel pin0 = null;
        DigitalChannel pin1 = null;
        AnalogInput analog_dist = null;
        AnalogInput analog_hsv = null;

        if (mode == "telemetry") {
            try {
                sensor0 = hardwareMap.get(RevColorSensorV3.class, "color");
            } catch (IllegalArgumentException e) {
                sensor0 = null;
            }
            sensor1 = hardwareMap.get(RevColorSensorV3.class, "color2");
        } else if (mode == "program") {
            try {
                crf0 = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color"));
            } catch (IllegalArgumentException e) {
                crf0 = null;
            }
            crf1 = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color2"));
        } else if (mode == "test") {
            //pin0 = hardwareMap.digitalChannel.get("artifact_color");
            //pin1 = hardwareMap.digitalChannel.get("artifact_distance");
            analog_dist = hardwareMap.analogInput.get("artifact_distance");
            analog_hsv = hardwareMap.analogInput.get("artifact_hsv");
        }

        while (opModeIsActive()) {
            telemetry.addData("mode", mode);

            if (sensor0 != null) {
                // read all 3 color channels in one I2C transmission:
                float[] hsv = new float[3];
                Color.RGBToHSV(sensor0.red(), sensor0.green(), sensor0.blue(), hsv);
                telemetry.addData("0 rgb: ", sensor0.red() + " " + sensor0.blue() + " " + sensor0.green());
                telemetry.addData("0 H  : ", hsv[0]);// + " " + hsv[1] + " " + hsv[2]);
                telemetry.addData("0 distance", sensor0.getDistance(DistanceUnit.MM));
            }
            if (sensor1 != null) {
                float[] hsv = new float[3];
                Color.RGBToHSV(sensor1.red(), sensor1.green(), sensor1.blue(), hsv);
                telemetry.addData("1 rgb: ", sensor1.red() + " " + sensor1.blue() + " " + sensor1.green());
                telemetry.addData("1 H  : ", hsv[0]);// + " " + hsv[1] + " " + hsv[2]);
                telemetry.addData("1 distance", sensor1.getDistance(DistanceUnit.MM));
            }
            if (crf1 != null) {
                /*
                // purple
                crf0.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 160 / 360.0 * 255, 190 / 360.0 * 255);
                //crf.setPin0Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 50);

                // distance
                //crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 140 / 360.0 * 255, 155 / 360.0 * 255);
                crf0.setPin1Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 50);
                */

                // set first sensor into analog mode, for distance
                crf0.setPin0Analog(ColorRangefinder.AnalogMode.DISTANCE);
                crf0.setPin1Digital(ColorRangefinder.DigitalMode.DISTANCE, 0, 100);

                // set second sensor into analog mode
                //crf1.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
                crf1.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
                crf1.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 100);

                // we only have to run this once, so stop the while loop
                break;
            }
            if (pin0 != null) {
                telemetry.addData("pin0", pin0.getState());
                telemetry.addData("pin1", pin1.getState());
            }
            if (analog_hsv != null) {
                telemetry.addData("analog HSV", (analog_hsv.getVoltage() / 3.3) * 360.0);
            }
            if (analog_dist != null) {
                telemetry.addData("analog distance", (analog_dist.getVoltage() / 3.3) * 100.0);
            }

            telemetry.update();
        }
    }
}

/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * Online documentation: <a href="https://docs.brushlandlabs.com">...</a>
 */
class ColorRangefinder {
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    /**
     * Configure Pin 0 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    /**
     * Configure Pin 1 to be in digital mode, and add a threshold.
     * Multiple thresholds can be added to the same pin by calling this function repeatedly.
     * For colors, bounds should be from 0-255, and for distance, bounds should be from 0-100 (mm).
     */
    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 0's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Sets the maximum distance (in millimeters) within which an object must be located for Pin 1's thresholds to trigger.
     * This is most useful when we want to know if an object is both close and the correct color.
     */
    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * Invert the hue value before thresholding it, meaning that the colors become their opposite.
     * This is useful if we want to threshold red; instead of having two thresholds we would invert
     * the color and look for blue.
     */
    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    /**
     * The denominator is what the raw sensor readings will be divided by before being scaled to 12-bit analog.
     * For the full range of that channel, leave the denominator as 65535 for colors or 100 for distance.
     * Smaller values will clip off higher ranges of the data in exchange for higher resolution within a lower range.
     */
    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    /**
     * Configure Pin 0 as analog output of one of the six data channels.
     * To read analog, make sure the physical switch on the sensor is flipped away from the
     * connector side.
     */
    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
            java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    /**
     * Save a brightness value of the LED to the sensor.
     *
     * @param value brightness between 0-255
     */
    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    /**
     * Change the I2C address at which the sensor will be found. The address can be reset to the
     * default of 0x52 by holding the reset button.
     *
     * @param value new I2C address from 1 to 127
     */
    public void setI2cAddress(int value) {
        i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    /**
     * Read distance via I2C
     *
     * @return distance in millimeters
     */
    public double readDistance() {
        java.nio.ByteBuffer bytes =
            java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    private void setDigital(
        PinNum pinNum,
        DigitalMode digitalMode,
        double lowerBound,
        double higherBound
        ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    // other writeable registers
    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
        return ((hue360 - 180) % 360);
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}


