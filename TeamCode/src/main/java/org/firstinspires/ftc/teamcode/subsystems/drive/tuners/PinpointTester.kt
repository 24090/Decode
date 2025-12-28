package org.firstinspires.ftc.teamcode.subsystems.drive.tuners

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer

@TeleOp(group = "Drive")
class PinpointTester: LinearOpMode() {

    override fun runOpMode() {
        val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
        pinpoint.setOffsets(
            Localizer.Companion.driveY,
            Localizer.Companion.strafeX,
            DistanceUnit.INCH
        )
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        pinpoint.setYawScalar(1.0)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.resetPosAndIMU()
        waitForStart()
        pinpoint.update()
        val initialY = pinpoint.encoderY
        val initialX = pinpoint.encoderX
        while (opModeIsActive()){
            pinpoint.update()
            telemetry.addData("pinpoint.position", pinpoint.position)
            telemetry.addData("Y encoder ticks", pinpoint.encoderY - initialY)
            telemetry.addData("X encoder ticks", pinpoint.encoderX - initialX)
            telemetry.update()
        }
    }
}