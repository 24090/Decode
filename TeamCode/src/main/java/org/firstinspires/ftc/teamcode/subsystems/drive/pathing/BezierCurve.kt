package org.firstinspires.ftc.teamcode.subsystems.drive.pathing
import org.firstinspires.ftc.teamcode.util.Interval
import org.firstinspires.ftc.teamcode.util.Polynomial
import org.firstinspires.ftc.teamcode.util.factorial
import kotlin.math.abs

class BezierCurve(vararg val controlPoints: Pose) {
    val degree = controlPoints.size - 1

    private fun calculatePolynomialCoefficient(j: Int) =
        controlPoints.withIndex().map {(i, controlPoint) ->
            controlPoint *
                    (when ((i + j)%2) { 0 -> 1; 1 -> -1 else -> throw UnsupportedOperationException()}) *
                    i.factorial()/(j-i).factorial()
        }.reduce(Pose::plus) *
        degree.factorial()/(degree - j).factorial()
    val xPolynomial = Polynomial(*DoubleArray(controlPoints.size) { j -> calculatePolynomialCoefficient(j).x})
    val yPolynomial = Polynomial(*DoubleArray(controlPoints.size) { j -> calculatePolynomialCoefficient(j).y})
    val headingPolynomial = Polynomial(*DoubleArray(controlPoints.size) { j -> calculatePolynomialCoefficient(j).heading})

    /**
     * Gets the rotational and translational acceleration at `t` as a `Pose`
     */
    fun getAcceleration(t: Double) = Pose(
        xPolynomial.derivative().derivative().evaluate(t),
        yPolynomial.derivative().derivative().evaluate(t),
        headingPolynomial.derivative().derivative().evaluate(t)
    )

    /**
     * Gets the rotational and translational velocity at `t` as a `Pose`
     */
    fun getVelocity(t: Double) = Pose(
        xPolynomial.derivative().evaluate(t),
        yPolynomial.derivative().evaluate(t),
        headingPolynomial.derivative().evaluate(t)
    )

    /**
     * Gets the heading and position at `t` as a `Pose`
     */
    fun getPose(t: Double) = Pose(
        xPolynomial.evaluate(t),
        yPolynomial.evaluate(t),
        headingPolynomial.evaluate(t)
    )

    fun closestT(position: Vector, guess: Double = 0.0) =
        // find when (x - x(t))^2 + (y - y(t))^2 is closest to 0
        ((xPolynomial + -position.x).pow(2) + (yPolynomial + -position.y).pow(2))
        .solveIA(Interval(0.0, 1.0))
        // prioritize solutions closer to guess (rarely relevant)
        .reduce { a: Double, b: Double -> if (abs(a - guess) <= abs(b - guess)) a else b }
}

fun CubicHermiteSpline(initialPose: Pose, initialVelocity:Pose, finalPose: Pose, finalVelocity: Pose) = BezierCurve(
    initialPose,
    initialPose + initialVelocity/3,
    finalPose - finalVelocity/3,
    finalPose
)
