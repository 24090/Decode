package org.firstinspires.ftc.teamcode.drivetrain

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

fun Pose(pose2d: Pose2D) = Pose(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS))

class Pose(var x: Double, var y: Double, var heading: Double) {
    fun inSquare(pose: Pose, xTolerance: Double, yTolerance: Double, headingTolerance: Double): Boolean{
        return (this - pose).inSquare(xTolerance, yTolerance, headingTolerance)
    }
    fun inSquare(xTolerance: Double, yTolerance: Double, headingTolerance: Double): Boolean{
        return  (this.x.absoluteValue < xTolerance) &&
                (this.y.absoluteValue < yTolerance) &&
                (this.heading.absoluteValue < headingTolerance)
    }
    fun inCircle(pose: Pose, distanceTolerance: Double, headingTolerance: Double): Boolean{
        return  (this - pose).inCircle(distanceTolerance, headingTolerance)
    }
    fun inCircle(distanceTolerance: Double, headingTolerance: Double): Boolean{
        return  (Vector.fromPose(this).length < distanceTolerance) &&
                ((this.heading) < headingTolerance)
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
        return "(x:" + "%.1f".format(x) +  "y:" + "%.1f".format(y) + "θ:" + "%.2f".format(heading) + ")"
    }
};


class Vector {
    val angle: Double

    // MUST BE POSITIVE!
    val length: Double

    private constructor(angle: Double, length: Double) {
        if (length < 0) {
            this.angle = AngleUnit.normalizeRadians(angle + PI)
            this.length = length.absoluteValue
        } else {
            this.angle = AngleUnit.normalizeRadians(angle)
            this.length = length.absoluteValue
        }
    }

    val x get() = length * cos(angle)
    val y get() = length * sin(angle)

    companion object {
        fun fromCartesian(x: Double, y: Double) = Vector(atan2(y, x), sqrt(x*x + y*y))
        fun fromPolar(angle: Double,  length: Double) = Vector(angle, length)
        fun fromPose(pose: Pose) = fromCartesian(pose.x, pose.y)
    }

    fun norm(): Vector = Vector(angle, 1.0);
    fun normalized(): Vector = Vector(angle, clamp(length, 0.0, 1.0))
    fun clampedLength(max: Double) = Vector(angle, clamp(length, 0.0, max))
    // dot product
    infix fun dot(v: Vector): Double {
        return v.x * this.x + v.y * this.y
    }

    operator fun times(x: Number): Vector {
        return Vector(angle, length * x.toDouble());
    }
    operator fun plus(v: Vector): Vector {
        return fromCartesian(x+v.x, y+v.y);
    }
    operator fun minus(v: Vector): Vector {
        return fromCartesian(x-v.x, y-v.y);
    }
    fun rotated(x: Number): Vector {
        return Vector(angle + x.toDouble(), length);
    }

    override fun toString(): String {
        return "[$x, $y]"
    }
}