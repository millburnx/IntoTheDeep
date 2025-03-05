package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.common.utils.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.common.utils.PinPoint.Companion.diameterMM
import org.firstinspires.ftc.teamcode.common.utils.PinPoint.Companion.ticksPerRevolution
import java.util.*

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */
@TeleOp(name = "goBILDA® PinPoint Odometry Example")
class SensorGoBildaPinpointExample : LinearOpMode() {
    val odo by lazy { hardwareMap["pinpoint"] as GoBildaPinpointDriver }

    var oldTime: Double = 0.0

    override fun runOpMode() {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0) // these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        val circumferenceMM = diameterMM * Math.PI
        val ticksPerMM = ticksPerRevolution / circumferenceMM

        odo.setEncoderResolution(ticksPerMM)

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
        )

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        odo.resetPosAndIMU()

        telemetry.addData("Status", "Initialized")
        telemetry.addData("X offset", odo.xOffset)
        telemetry.addData("Y offset", odo.yOffset)
        telemetry.addData("Device Version Number:", odo.deviceVersion)
        telemetry.addData("Device Scalar", odo.yawScalar)
        telemetry.update()

        // Wait for the game to start (driver presses START)
        waitForStart()
        resetRuntime()

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // full read
            odo.update()

            // only update heading, this is faster
            // odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
            if (gamepad1.a) {
                odo.resetPosAndIMU() // resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b) {
                odo.recalibrateIMU() // recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            val newTime = runtime
            val loopTime = newTime - oldTime
            val frequency = 1 / loopTime
            oldTime = newTime

            val pos = odo.position
            val data =
                String.format(
                    Locale.US,
                    "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.INCH),
                    pos.getY(DistanceUnit.INCH),
                    pos.getHeading(
                        AngleUnit.DEGREES,
                    ),
                )
            telemetry.addData("Position", data)

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            val vel = odo.velocity
            val velocity =
                String.format(
                    Locale.US,
                    "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    vel.getX(DistanceUnit.INCH),
                    vel.getY(DistanceUnit.INCH),
                    vel.getHeading(
                        AngleUnit.DEGREES,
                    ),
                )
            telemetry.addData("Velocity", velocity)

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
             */
            telemetry.addData("Status", odo.getDeviceStatus())

            telemetry.addData(
                "Pinpoint Frequency",
                odo.frequency,
            ) // prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency) // prints the control system refresh rate
            telemetry.update()
        }
    }
}
