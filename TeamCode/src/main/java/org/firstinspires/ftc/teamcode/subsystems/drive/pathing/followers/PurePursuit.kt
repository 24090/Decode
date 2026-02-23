package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers

import android.util.Log
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getTranslationalVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.BezierCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.subsystems.reads.VoltageReader.controlHubVoltage
import kotlin.math.max

sealed class HeadingBehaviour{
    class Tangent(val angle: Double): HeadingBehaviour()
    object Snap: HeadingBehaviour()
    object Interpolate: HeadingBehaviour()
}

fun getPurePursuit(path: PurePursuitPath, localizer: Localizer): () -> DriveVectors{
    return {
        purePursuit(path, localizer.pose, localizer.poseVel)
    }
}

fun purePursuit(path: PurePursuitPath, pose: Pose, velocity: Pose): DriveVectors{
    val followPoint = path.getFollowPoint(pose.vector(), 48.0)
    Log.i("follow point", followPoint.toString())
    return pointToPoint(pose, velocity, followPoint)
}

fun getFollowCurve(curve: BezierCurve, localizer: Localizer) = { followCurve(curve, localizer.pose, localizer.poseVel) }
fun followCurve(curve: BezierCurve, pose: Pose, velocity: Pose): DriveVectors{
    // 1. corrective position/heading
    // 2. static friction feedforward
    // 3. accelvelstuff

    val t = curve.closestT(pose.vector())
    println(t)
    val curvePose = curve.getPose(t)
    val curveVelocity = curve.getVelocity(t).vector()
    val curveAccel = curve.getAcceleration(t).vector()
    val corrective = pointToPoint(pose, velocity, curvePose, false)
    val static = curveVelocity.normalized() * (kS + 0.4)
    val goodVelocityLength = max((velocity.vector() dot curveVelocity.norm()), 0.0)
    val accelvel = (Vector.fromPolar(
        angle = curveAccel.angle,
        length = goodVelocityLength * (curveAccel.length / curveVelocity.length.let { if (it == 0.0) 0.001 else it} )
    ) + Vector.fromPolar(
        angle = curveVelocity.angle,
        goodVelocityLength,
    )) * 2.0

    return corrective
        .addWithoutPriority(getTranslationalVectors(static.x, static.y), controlHubVoltage / 14.0)
        .driveAccelCorrected(
            tipAccelBackward, tipAccelForward,
            kS, kV, kA,
            getRelativeVelocity(pose, velocity).x
        )

}
