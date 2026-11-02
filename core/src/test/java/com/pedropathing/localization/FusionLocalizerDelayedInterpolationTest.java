/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.lang.reflect.Field;
import java.util.NavigableMap;
import org.junit.jupiter.api.Test;

class FusionLocalizerDelayedInterpolationTest {
    @Test
    void exactDelayedArcMeasurementsDoNotMoveTheLatestPoseInEitherTurnDirection() throws ReflectiveOperationException {
        assertArcMeasurementDoesNotMoveEndpoint(Pose.zero(), Math.PI / 2);
        assertArcMeasurementDoesNotMoveEndpoint(Pose.zero(), -Math.PI / 2);
    }

    @Test
    void interpolationHandlesTranslatedRotatedOriginsAndHeadingWraparound() throws ReflectiveOperationException {
        assertArcMeasurementDoesNotMoveEndpoint(new Pose(20, -10, Math.toRadians(350)), Math.toRadians(20));
    }

    private void assertArcMeasurementDoesNotMoveEndpoint(Pose origin, double angle)
            throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.setPose(origin);
        raw.sample(origin, Velocity.zero());
        localizer.update();
        long lower = timestamp(localizer);
        double radius = Math.copySign(10, angle);
        Pose endpoint = origin.compose(new Pose(radius * Math.sin(angle), radius * (1 - Math.cos(angle)), angle));
        raw.sample(endpoint, Velocity.zero());
        localizer.update();
        long upper = timestamp(localizer);
        long middle = lower + (upper - lower) / 2;
        assertTrue(middle > lower && middle < upper);
        double ratio = (double) (middle - lower) / (upper - lower);
        double partialAngle = angle * ratio;
        Pose measured = origin.compose(
                new Pose(radius * Math.sin(partialAngle), radius * (1 - Math.cos(partialAngle)), partialAngle));

        localizer.addMeasurement(measured, middle);
        assertPose(endpoint, localizer.pose());
        localizer.addMeasurement(measured, middle);
        assertPose(endpoint, localizer.pose());
    }

    @Test
    void aZeroInnovationAtTheInterpolatedPredictionCannotChangeTheEndpoint() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        long lower = timestamp(localizer);
        Pose endpoint = new Pose(10, 10, Math.PI / 2);
        raw.sample(endpoint, Velocity.zero());
        localizer.update();
        long upper = timestamp(localizer);
        long middle = lower + (upper - lower) / 2;
        java.lang.reflect.Method interpolate = FusionLocalizer.class.getDeclaredMethod("interpolate", long.class);
        interpolate.setAccessible(true);
        Object prediction = interpolate.invoke(localizer, middle);
        Pose zeroInnovationMeasurement = (Pose) field(prediction, "pose");

        localizer.addMeasurement(zeroInnovationMeasurement, middle);

        assertPose(endpoint, localizer.pose());
    }

    @Test
    void straightSegmentsAndExactSampleTimesRemainUnchanged() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        long lower = timestamp(localizer);
        Pose endpoint = new Pose(10, 0, 0);
        raw.sample(endpoint, Velocity.zero());
        localizer.update();
        long upper = timestamp(localizer);
        long middle = lower + (upper - lower) / 2;
        double ratio = (double) (middle - lower) / (upper - lower);

        localizer.addMeasurement(new Pose(10 * ratio, 0, 0), middle);
        assertPose(endpoint, localizer.pose());
        localizer.addMeasurement(endpoint, upper);
        assertPose(endpoint, localizer.pose());
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
