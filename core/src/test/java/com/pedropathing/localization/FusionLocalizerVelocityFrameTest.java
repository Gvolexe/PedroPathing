/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Velocity;
import java.lang.reflect.Field;
import java.util.NavigableMap;
import org.junit.jupiter.api.Test;

class FusionLocalizerVelocityFrameTest {
    @Test
    void visionHeadingCorrectionImmediatelyRotatesWorldVelocityAndPreservesBodyVelocity()
            throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        raw.sample(Pose.zero(), new Velocity(10, 2, 0.5));
        localizer.update();

        localizer.addMeasurement(
                new Pose(Double.NaN, Double.NaN, Math.PI / 2), timestamp(localizer), new Pose(1, 1, 1e-6));

        assertVelocityMatchesBody(localizer, new Twist(10, 2, 0.5));
    }

    @Test
    void subsequentOdometryReportsVelocityInTheSameFrameAsItsPositionIncrement() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        localizer.update();
        localizer.addMeasurement(
                new Pose(Double.NaN, Double.NaN, Math.PI / 2), timestamp(localizer), new Pose(1, 1, 1e-6));
        Pose before = localizer.pose();
        raw.sample(new Pose(1, 0, 0), new Velocity(10, 0, 0));

        localizer.update();

        assertVelocityMatchesBody(localizer, new Twist(10, 0, 0));
        assertEquals((localizer.pose().x() - before.x()) * 10, localizer.velocity().vx, TOLERANCE);
        assertEquals((localizer.pose().y() - before.y()) * 10, localizer.velocity().vy, TOLERANCE);
    }

    @Test
    void delayedHeadingCorrectionReframesReplayedSamplesAndTheLatestVelocity() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        raw.sample(Pose.zero(), new Velocity(10, 0, 0));
        localizer.update();
        long earlier = timestamp(localizer);
        raw.sample(new Pose(1, 0, 0), new Velocity(10, 0, 0));
        localizer.update();

        localizer.addMeasurement(new Pose(Double.NaN, Double.NaN, Math.PI / 2), earlier, new Pose(1, 1, 1e-6));

        assertVelocityMatchesBody(localizer, new Twist(10, 0, 0));
        assertEquals(localizer.pose().x() * 10, localizer.velocity().vx, TOLERANCE);
        assertEquals(localizer.pose().y() * 10, localizer.velocity().vy, TOLERANCE);
    }

    @Test
    void explicitCorrectionsAndTeleportsPreserveBodyVelocity() throws ReflectiveOperationException {
        RawLocalizer raw = new RawLocalizer();
        FusionLocalizer localizer = create(raw);
        raw.sample(Pose.zero(), new Velocity(10, 2, 0.5));
        localizer.update();
        Twist expected = new Twist(10, 2, 0.5);

        localizer.correctPose(new Pose(2, 3, Math.PI / 2));
        assertVelocityMatchesBody(localizer, expected);
        localizer.addMeasurement(localizer.pose(), timestamp(localizer));
        assertVelocityMatchesBody(localizer, expected);
        localizer.setPose(new Pose(4, 5, Math.PI));
        assertVelocityMatchesBody(localizer, expected);
    }

    private static void assertVelocityMatchesBody(FusionLocalizer localizer, Twist body) {
        Velocity expected = body.toVelocity(localizer.pose().heading());
        assertEquals(expected.vx, localizer.velocity().vx, TOLERANCE);
        assertEquals(expected.vy, localizer.velocity().vy, TOLERANCE);
        assertEquals(expected.omega, localizer.velocity().omega, TOLERANCE);
        assertEquals(body.vx, localizer.twist().vx, TOLERANCE);
        assertEquals(body.vy, localizer.twist().vy, TOLERANCE);
        assertEquals(body.omega, localizer.twist().omega, TOLERANCE);
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
