package org.firstinspires.ftc.teamcode.util

import kotlin.math.max
import kotlin.math.pow

class Polynomial(vararg val coefficients: Double) {
    val length = coefficients.size
    val degree = coefficients.size - 1
    fun evaluate(value: Double): Double{
        return coefficients.withIndex().sumOf { (i, coefficient) -> coefficient * value.pow(i) }
    }
    fun derivative(): Polynomial{
        return Polynomial(*coefficients.withIndex().map {(i,coefficient) -> coefficient*(degree-i)}.toDoubleArray())
    }
    fun solveIA(interval: Interval, iterations: Int = 50, first: Boolean = true): List<Double>{
        val out = coefficients.withIndex().map { (i, coefficient) ->  interval.pow(i) * coefficient}.reduce(Interval::plus)
        if (!out.contains(0.0)) {
            if (first) {
                val closest = if (out.upper < 0.0) out.upper else out.lower
                val newConstant = coefficients.last() - closest
                println("Closest to zero: $closest")
                val newPolynomial = Polynomial(*coefficients.dropLast(1).plus(newConstant).toDoubleArray())
                return newPolynomial.solveIA(interval, iterations)
            }
            return listOf()
        }
        val pivot = (interval.lower + interval.upper)/2
        if (iterations == 1) {
            return listOf(pivot)
        }
        return solveIA(
            Interval(interval.lower, pivot),
            iterations - 1,
            first = false
        ) + solveIA(
            Interval(pivot, interval.upper),
            iterations - 1,
            first = false
        )
    }

    operator fun plus(other: Double): Polynomial {
        return Polynomial(
            *DoubleArray(max(this.length, 1)){ i ->
                if (i < this.length) coefficients[i] else 0.0 +
                if (i < 1) other else 0.0
            }
        )
    }

    operator fun plus(other: Polynomial): Polynomial {
        return Polynomial(
            *DoubleArray(max(this.length, other.length)){ i ->
                if (i < this.length) coefficients[i] else 0.0 +
                if (i < other.length) other.coefficients[i] else 0.0
            }
        )
    }

    operator fun times(other: Double) = Polynomial(
        *DoubleArray(this.length * 1){ i -> this.coefficients[i] * other }
    )

    operator fun times(other: Polynomial): Polynomial {
        val newCoefficients = DoubleArray(this.length * other.length)
        for (i in 0..<this.length) {
            for (j in 0..<other.length) {
                newCoefficients[(i+1) * (j+1) - 1] += this.coefficients[i] * other.coefficients[j]
            }
        }
        return Polynomial(*newCoefficients)
    }

    fun pow(n: Int): Polynomial {
        var v = Polynomial(1.0)
        for (i in 1..n) {
            v = v*this
        }
        return v
    }
}