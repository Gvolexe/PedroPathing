/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.paths.interpolator;

import static com.pedropathing.api.Paths.curve;
import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import org.junit.jupiter.api.Test;

class ReversedGeometricInterpolatorTest {
    @Test
    void reversedTangentUsesTheOriginalCurveParameterInsidePiecewiseSections() {
        Path shape = curve(new Pose(0, 0), new Pose(24, 0), new Pose(24, 24));
        Path reversed = shape.heading(Interpolator.piecewise()
                .until(0.5, Interpolator.tangent.reverse())
                .until(1.0, Interpolator.tangent.reverse()));
        Path tangent = shape.tangent();

        for (double t : new double[] {0.1, 0.499, 0.501, 0.9}) {
            double expected = Angle.normalize(tangent.heading(t) + Math.PI);
            assertEquals(0, Angle.normalizeSigned(reversed.heading(t) - expected), 1e-9);
        }
    }

    @Test
    void reversedFacingPointRemainsGeometricOnAnUnevenCurve() {
        Path shape = curve(new Pose(0, 0), new Pose(48, 0), new Pose(48, 6));
        Interpolator facing = Interpolator.facingPoint(Vector2D.cartesian(0, 24));
        Path expected = shape.heading(facing);
        Path direct = shape.heading(facing.reverse());
        Path split = shape.heading(
                Interpolator.piecewise().until(0.5, facing.reverse()).until(1.0, facing.reverse()));

        for (double t : new double[] {0.1, 0.49, 0.51, 0.9}) {
            double heading = Angle.normalize(expected.heading(t) + Math.PI);
            assertEquals(0, Angle.normalizeSigned(direct.heading(t) - heading), 1e-9);
            assertEquals(0, Angle.normalizeSigned(split.heading(t) - heading), 1e-9);
        }
    }

    @Test
    void reversedLinearStillUsesProgressWithinEachPiece() {
        Interpolator linear = Interpolator.linear(0, Math.PI / 2);
        Path shape = curve(new Pose(0, 0), new Pose(24, 0), new Pose(24, 24));
        Path split = shape.heading(
                Interpolator.piecewise().until(0.5, linear.reverse()).until(1.0, Interpolator.constant(Math.PI * 1.5)));
        double t = 0.25;
        double sectionProgress = shape.curve.pathCompletion(t) / 0.5;

        assertFalse(linear.reverse().isGeometric());
        assertEquals(
                0,
                Angle.normalizeSigned(
                        split.heading(t) - Angle.normalize(linear.interpolate(shape.curve, sectionProgress) + Math.PI)),
                1e-9);
    }
}
