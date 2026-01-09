package org.firstinspires.ftc.teamcode.subsystems.drive.pathing

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Vector {
    val angle: Double
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
        fun fromPolar(angle: Double, length: Double) = Vector(angle, length)
        fun fromPose(pose: Pose) = fromCartesian(pose.x, pose.y)
    }

    fun norm(): Vector = Vector(angle, 1.0)
    fun normalized(): Vector = Vector(angle, clamp(length, 0.0, 1.0))
    fun clampedLength(max: Double) = Vector(angle, clamp(length, 0.0, max))
    // dot product
    infix fun dot(v: Vector): Double {
        return v.x * this.x + v.y * this.y
    }

    fun scalarProjection(v: Vector) = v.length * cos(this.angle - v.angle)

    // scalar inverse projection of v onto this
    fun scalarInverseProjection(v: Vector) =
        cos(this.angle - v.angle).let { if (it != 0.0) v.length/it else Double.POSITIVE_INFINITY
        }

    // component of v parallel to this
    fun projection(v: Vector) = this.norm() * (v dot this.norm())

    // component of v perpendicular to this
    fun rejection(v: Vector) =  v - projection(v)

    operator fun times(x: Number): Vector {
        return Vector(angle, length * x.toDouble())
    }
    operator fun plus(v: Vector): Vector {
        return fromCartesian(x+v.x, y+v.y)
    }
    operator fun minus(v: Vector): Vector {
        return fromCartesian(x-v.x, y-v.y)
    }
    fun rotated(x: Number): Vector {
        return Vector(angle + x.toDouble(), length)
    }

    override fun toString(): String {
        return "[$x, $y]"
    }

    fun mirrored() = fromCartesian(this.x, -this.y)

    fun mirroredIf(v: Boolean) = if (v) this.mirrored() else this
}

fun getRelativePosition(zeroPose: Pose, fieldPosition: Vector) =
    (fieldPosition - zeroPose.vector()).rotated(-zeroPose.heading)


fun getRelativePose(zeroPose: Pose, fieldPose: Pose) =
    getRelativePosition(zeroPose, fieldPose.vector())
        .let {
            Pose(it.x, it.y, AngleUnit.normalizeRadians(fieldPose.heading - zeroPose.heading))
        }

fun getRelativeVelocity(zeroPose: Pose, fieldVelocity: Pose) =
    Vector.fromPose(fieldVelocity).rotated(-zeroPose.heading).let {
        Pose(it.x, it.y, fieldVelocity.heading)
    }