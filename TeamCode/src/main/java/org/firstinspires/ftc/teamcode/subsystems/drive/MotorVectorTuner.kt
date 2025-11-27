package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer.Companion.driveY
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer.Companion.strafeX

@TeleOp(group = "Drive")
class MotorVectorTuner(): LinearOpMode() {
    override fun runOpMode() {
        val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
        pinpoint.setOffsets(driveY, strafeX, INCH)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        pinpoint.setYawScalar(0.0)
        pinpoint.position = Pose2D(INCH, 0.0, 0.0, AngleUnit.RADIANS, 0.0)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.update()
        val names = arrayListOf("fl", "bl", "fr", "br")
        var n = 0
        val xInitial = pinpoint.encoderX
        val yInitial = pinpoint.encoderY
        while (opModeInInit()){
            if (gamepad1.xWasPressed()) {
                n = (n+1)%4
            }
            if (gamepad1.yWasPressed()) {
                n = (n-1)%4
            }
            telemetry.addData("motor", names[n])
            telemetry.update()
        }
        waitForStart()
        val motorName = names[n]
        val motor = hardwareMap.get(DcMotor::class.java, motorName)
        if (n < 2) motor.direction = DcMotorSimple.Direction.REVERSE
        motor.power = 1.0
        sleep(2000)
        motor.power = 0.0
        pinpoint.update()
        telemetry.addData("motor", motorName)
        telemetry.addData("x", pinpoint.getPosX(INCH))
        telemetry.addData("y", pinpoint.getPosY(INCH))
        telemetry.update()
        while (opModeIsActive()){}
    }
}