package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose

class Localizer(hwMap: HardwareMap) {
    var pinpoint: GoBildaPinpointDriver = hwMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
    companion object {
        @JvmStatic var driveY: Double = -2.4
        @JvmStatic var strafeX: Double = -2.68
    }
    init {
        // X and Y are INTENTIONALLY swapped
        pinpoint.setOffsets(driveY, strafeX, INCH)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setYawScalar(1.0)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED)
        setDefaultBulkreadScope()
        pinpoint.recalibrateIMU()
        pinpoint.update()
    }
    var pose
        get() = Pose(pinpoint.position)
        set(v){ pinpoint.position = Pose2D(INCH, v.x, v.y, RADIANS, v.heading) }

    val x
        get() = pose.x
    val y
        get() = pose.y
    val heading
        get() = pose.heading

    val poseVel
        get() = Pose(xVel, yVel, headingVel)
    val xVel
        get() = pinpoint.getVelX(INCH)
    val yVel
        get() =  pinpoint.getVelY(INCH)
    val headingVel
        get() = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)

    fun setDefaultBulkreadScope() {
        pinpoint.setBulkReadScope(
            GoBildaPinpointDriver.Register.X_VELOCITY,
            GoBildaPinpointDriver.Register.Y_VELOCITY,
            GoBildaPinpointDriver.Register.H_VELOCITY,
            GoBildaPinpointDriver.Register.X_POSITION,
            GoBildaPinpointDriver.Register.Y_POSITION,
            GoBildaPinpointDriver.Register.H_ORIENTATION,
        )
    }

    fun setWheelieBulkreadScope() {
        pinpoint.setBulkReadScope(
            GoBildaPinpointDriver.Register.X_VELOCITY,
            GoBildaPinpointDriver.Register.Y_VELOCITY,
            GoBildaPinpointDriver.Register.H_VELOCITY,
            GoBildaPinpointDriver.Register.X_POSITION,
            GoBildaPinpointDriver.Register.Y_POSITION,
            GoBildaPinpointDriver.Register.H_ORIENTATION,
            GoBildaPinpointDriver.Register.PITCH
        )
    }
}