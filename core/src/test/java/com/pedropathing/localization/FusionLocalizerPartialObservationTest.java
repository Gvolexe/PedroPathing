/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.lang.reflect.Field;
import java.util.NavigableMap;
import org.junit.jupiter.api.Test;

class FusionLocalizerPartialObservationTest {
    @Test
    void xOnlyObservationUsesCorrelatedCovarianceWithoutObservingY() throws ReflectiveOperationException {
        assertSingleAxisObservation(0);
    }

    @Test
    void yOnlyObservationUsesCorrelatedCovarianceWithoutObservingX() throws ReflectiveOperationException {
        assertSingleAxisObservation(1);
    }

    private void assertSingleAxisObservation(int axis) throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        // Deterministic covariance isolates the observation model from wall-clock process-noise integration.
        Matrix covariance = new Matrix(new double[][] {{2, 1, 0}, {1, 2, 0}, {0, 0, 1}});
        setCovariance(localizer, covariance);
        Pose measurement = axis == 0 ? new Pose(1, Double.NaN, Double.NaN) : new Pose(Double.NaN, 1, Double.NaN);

        localizer.addMeasurement(measurement, timestamp(localizer));

        assertEquals(axis == 0 ? 2.0 / 3 : 1.0 / 3, localizer.pose().x(), TOLERANCE);
        assertEquals(axis == 0 ? 1.0 / 3 : 2.0 / 3, localizer.pose().y(), TOLERANCE);
        assertEquals(0, localizer.pose().heading(), TOLERANCE);
        Matrix posterior = (Matrix) field(localizer, "P");
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double expected = covariance.get(row, col) - covariance.get(row, axis) * covariance.get(axis, col) / 3;
                assertEquals(expected, posterior.get(row, col), TOLERANCE);
            }
        }
    }

    @Test
    void headingOnlyObservationCanCorrectCorrelatedTranslation() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        setCovariance(localizer, new Matrix(new double[][] {{2, 0, 0.5}, {0, 2, 0}, {0.5, 0, 1}}));

        localizer.addMeasurement(new Pose(Double.NaN, Double.NaN, 1), timestamp(localizer));

        assertPose(new Pose(0.25, 0, 0.5), localizer.pose());
    }

    @Test
    void fullAndEmptyObservationsRetainTheirExpectedBehavior() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        localizer.addMeasurement(new Pose(3, 3, 1), timestamp(localizer));
        assertPose(new Pose(2, 2, 0.5), localizer.pose());
        Matrix before = (Matrix) field(localizer, "P");

        localizer.addMeasurement(new Pose(Double.NaN, Double.NaN, Double.NaN), timestamp(localizer));

        assertPose(new Pose(2, 2, 0.5), localizer.pose());
        Matrix after = (Matrix) field(localizer, "P");
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                assertEquals(before.get(row, col), after.get(row, col), TOLERANCE);
            }
        }
    }

    private static void setCovariance(FusionLocalizer localizer, Matrix covariance)
            throws ReflectiveOperationException {
        Field current = FusionLocalizer.class.getDeclaredField("P");
        current.setAccessible(true);
        current.set(localizer, covariance);
        Object sample = history(localizer).lastEntry().getValue();
        Field historical = sample.getClass().getDeclaredField("covariance");
        historical.setAccessible(true);
        historical.set(sample, covariance);
    }

    private static final double TOLERANCE = 1e-8;

    private static class RawLocalizer implements Localizer {
        private MotionState state = MotionState.zero();

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void update() {}

        @Override
        public void setPose(Pose pose) {
            state = state.withPose(pose);
        }

        @Override
        public void reset() {
            // Model devices whose cached state is not refreshed until update().
        }

        void sample(Pose pose, Velocity velocity) {
            state = MotionState.ofVelocity(pose, velocity);
        }
    }

    private static FusionLocalizer create(RawLocalizer raw) {
        return new FusionLocalizer(raw, new Pose(2, 2, 1), new Pose(2, 0.1, 0.1), new Pose(1, 1, 1), 100);
    }

    private static Object field(Object object, String name) throws ReflectiveOperationException {
        Field field = object.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.get(object);
    }

    @SuppressWarnings("unchecked")
    private static NavigableMap<Long, Object> history(FusionLocalizer localizer) throws ReflectiveOperationException {
        return (NavigableMap<Long, Object>) field(localizer, "history");
    }

    private static long timestamp(FusionLocalizer localizer) throws ReflectiveOperationException {
        return history(localizer).lastKey();
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.x(), actual.x(), TOLERANCE);
        assertEquals(expected.y(), actual.y(), TOLERANCE);
        assertEquals(0, com.pedropathing.utils.Angle.normalizeSigned(actual.heading() - expected.heading()), TOLERANCE);
    }
}
