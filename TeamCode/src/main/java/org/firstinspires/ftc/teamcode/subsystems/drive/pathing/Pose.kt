package org.firstinspires.ftc.teamcode.subsystems.drive.pathing

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyT
import kotlin.math.absoluteValue

fun Pose(pose2d: Pose2D) = Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS))

class Pose(var x: Double, var y: Double, var heading: Double) {
    fun inSquare(pose: Pose, xTolerance: Double, yTolerance: Double, headingTolerance: Double): Boolean{
        return (this - pose).inSquare(xTolerance, yTolerance, headingTolerance)
    }
    fun inSquare(xTolerance: Double = xyT, yTolerance: Double = xyT, headingTolerance: Double = hT): Boolean{
        return  (this.x.absoluteValue < xTolerance) &&
                (this.y.absoluteValue < yTolerance) &&
                (this.heading.absoluteValue < headingTolerance)
    }
    fun inCircle(pose: Pose, distanceTolerance: Double = xyT, headingTolerance: Double = hT): Boolean{
        return  (this - pose).inCircle(distanceTolerance, headingTolerance)
    }
    fun inCircle(distanceTolerance: Double, headingTolerance: Double): Boolean{
        return  (Vector.fromPose(this).length < distanceTolerance) &&
                (this.heading.absoluteValue < headingTolerance)
    }

    operator fun plus(v: Pose): Pose{
        return Pose(x + v.x, y + v.y, heading+v.heading)
    }

    operator fun minus(v: Pose): Pose{
        return Pose(x - v.x, y - v.y, heading - v.heading)
    }

    operator fun times(v: Number): Pose{
        return Pose(x * v.toDouble(), y * v.toDouble(), heading * v.toDouble())
    }

    operator fun div(v: Number): Pose{
        return Pose(x / v.toDouble(), y / v.toDouble(), heading / v.toDouble())
    }

    operator fun unaryMinus(): Pose{
        return Pose(-x, -y, -heading)
    }

    override fun toString(): String {
        return "(x:" + "%.1f".format(x) +  " y:" + "%.1f".format(y) + " θ:" + "%.2f".format(heading) + ")"
    }

    fun mirrored() = Pose(this.x, -this.y, -this.heading)

    fun mirroredIf(v: Boolean) = if (v) this.mirrored() else this

    fun vector() = Vector.fromPose(this)
}