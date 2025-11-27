package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth

class Localizer(hwMap: HardwareMap) {
    var pinpoint: GoBildaPinpointDriver = hwMap.get(GoBildaPinpointDriver::class.java, "pinpoint")
    companion object {
        @JvmStatic var driveY: Double = -6.1
        @JvmStatic var strafeX: Double = -2.64
    }
    init {
        // X and Y are INTENTIONALLY swapped
        pinpoint.setOffsets(driveY, strafeX, INCH)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
        pinpoint.setYawScalar(1.0)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.resetPosAndIMU()
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

    fun fieldPoseToRelative(fieldPose: Pose): Pose {
        val translation = fieldVecToRelative(Vector.fromPose(fieldPose))
        return Pose(
            translation.x,
            translation.y,
            AngleUnit.normalizeRadians(fieldPose.heading - heading)
        )
    }

    fun fieldVecToRelative(fieldVec: Vector): Vector =
        (Vector.fromCartesian(fieldVec.x, fieldVec.y) - Vector.fromCartesian(x, y)).rotated(-heading)

    fun relativePoseToField(relativePose: Pose): Pose {
        val translation = Vector.fromCartesian(relativePose.x, relativePose.y).rotated(heading) + Vector.fromCartesian(x, y)
        return Pose(
            translation.x,
            translation.y,
            AngleUnit.normalizeRadians(heading + relativePose.heading)
        )
    }

}