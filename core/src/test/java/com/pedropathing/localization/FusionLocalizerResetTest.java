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

class FusionLocalizerResetTest {
    @Test
    void resetClearsTheFusedStateEvenWhenTheWrappedResetLeavesCachedState() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        raw.sample(new Pose(10, 0, 0), new Velocity(5, 0, 0));
        localizer.update();
        long oldTimestamp = timestamp(localizer);
        localizer.addMeasurement(new Pose(13, 0, 0), oldTimestamp);
        assertEquals(12, localizer.pose().x(), TOLERANCE);

        localizer.reset();

        assertPose(Pose.zero(), localizer.pose());
        assertPose(Pose.zero(), raw.pose());
        assertEquals(0, localizer.velocity().linSpeed(), TOLERANCE);
        assertEquals(0, localizer.velocity().omega, TOLERANCE);
        assertEquals(1, history(localizer).size());
        localizer.addMeasurement(new Pose(100, 100, 1), oldTimestamp);
        assertPose(Pose.zero(), localizer.pose());

        raw.sample(Pose.zero(), Velocity.zero());
        localizer.update();
        assertPose(Pose.zero(), localizer.pose());
    }

    @Test
    void resetRestoresInitialCovarianceAndRebasesTime() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        localizer.addMeasurement(new Pose(3, 0, 0), timestamp(localizer));
        localizer.addMeasurement(new Pose(3, 0, 0), timestamp(localizer));

        localizer.reset();
        // The first sample after reset must not integrate process noise over the previous session's time.
        raw.sample(Pose.zero(), new Velocity(100, 100, 0));
        localizer.update();
        localizer.addMeasurement(new Pose(3, 0, 0), timestamp(localizer));

        assertEquals(2, localizer.pose().x(), TOLERANCE);
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
