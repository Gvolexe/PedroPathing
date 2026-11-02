/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.revhub.localizers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Test;

public class EncoderLocalizerResetTest {
    @Test
    public void twoWheelResetImmediatelyClearsPoseAndVelocity() {
        Fixture f = new Fixture();
        TwoWheelConfig config = new TwoWheelConfig(c -> {
            c.xPodName.set("left");
            c.yPodName.set("strafe");
            c.imuName.set("imu");
            c.imu.set(f.imu);
            c.xPodOffset.set(0.0);
            c.yPodOffset.set(0.0);
            c.forwardTicksToInches.set(0.01);
            c.strafeTicksToInches.set(0.02);
            c.xPodDirection.set(Encoder.FORWARD);
            c.yPodDirection.set(Encoder.FORWARD);
        });

        assertResetClearsState(new TwoWheelLocalizer(f.map, config), f);
    }

    @Test
    public void threeWheelResetImmediatelyClearsPoseAndVelocity() {
        Fixture f = new Fixture();
        ThreeWheelConfig config = new ThreeWheelConfig(c -> {
            c.leftEncoderName.set("left");
            c.rightEncoderName.set("right");
            c.strafeEncoderName.set("strafe");
            c.leftPodY.set(5.0);
            c.rightPodY.set(-5.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(0.01);
            c.strafeTicksToInches.set(0.02);
            c.turnTicksToRadians.set(0.01);
            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.rightEncoderDirection.set(Encoder.FORWARD);
            c.strafeEncoderDirection.set(Encoder.FORWARD);
        });

        ThreeWheelLocalizer localizer = new ThreeWheelLocalizer(f.map, config);
        assertResetClearsState(localizer, f);
        assertEquals(0.0, localizer.getTotalHeading(), 0.0);
    }

    @Test
    public void threeWheelImuResetImmediatelyClearsPoseAndVelocity() {
        Fixture f = new Fixture();
        ThreeWheelIMUConfig config = new ThreeWheelIMUConfig(c -> {
            c.leftEncoderName.set("left");
            c.rightEncoderName.set("right");
            c.strafeEncoderName.set("strafe");
            c.imuName.set("imu");
            c.imu.set(f.imu);
            c.leftPodY.set(5.0);
            c.rightPodY.set(-5.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(0.01);
            c.strafeTicksToInches.set(0.02);
            c.turnTicksToRadians.set(0.01);
            c.leftEncoderDirection.set(Encoder.FORWARD);
            c.rightEncoderDirection.set(Encoder.FORWARD);
            c.strafeEncoderDirection.set(Encoder.FORWARD);
        });

        ThreeWheelIMULocalizer localizer = new ThreeWheelIMULocalizer(f.map, config);
        assertResetClearsState(localizer, f);
        assertEquals(0.0, localizer.getTotalHeading(), 0.0);
    }

    private static void assertResetClearsState(Localizer localizer, Fixture fixture) {
        localizer.setPose(new Pose(10, 20, 0.5));
        fixture.left.set(100);
        fixture.right.set(150);
        fixture.strafe.set(50);
        localizer.update();
        assertTrue(localizer.velocity().linSpeed() > 0);

        localizer.reset();

        assertZeroState(localizer);
        localizer.update();
        assertZeroState(localizer);
    }

    private static void assertZeroState(Localizer localizer) {
        assertEquals(0.0, localizer.pose().x(), 0.0);
        assertEquals(0.0, localizer.pose().y(), 0.0);
        assertEquals(0.0, localizer.pose().heading(), 0.0);
        assertEquals(0.0, localizer.velocity().vx, 0.0);
        assertEquals(0.0, localizer.velocity().vy, 0.0);
        assertEquals(0.0, localizer.velocity().omega, 0.0);
        assertEquals(0.0, localizer.twist().vx, 0.0);
        assertEquals(0.0, localizer.twist().vy, 0.0);
        assertEquals(0.0, localizer.twist().omega, 0.0);
    }

    private static final class Fixture {
        final HardwareMap map = mock(HardwareMap.class);
        final CustomIMU imu = mock(CustomIMU.class);
        final AtomicInteger left = new AtomicInteger();
        final AtomicInteger right = new AtomicInteger();
        final AtomicInteger strafe = new AtomicInteger();

        Fixture() {
            DcMotorEx leftMotor = motor(left);
            DcMotorEx rightMotor = motor(right);
            DcMotorEx strafeMotor = motor(strafe);
            when(map.get(DcMotorEx.class, "left")).thenReturn(leftMotor);
            when(map.get(DcMotorEx.class, "right")).thenReturn(rightMotor);
            when(map.get(DcMotorEx.class, "strafe")).thenReturn(strafeMotor);
        }

        private DcMotorEx motor(AtomicInteger ticks) {
            DcMotorEx motor = mock(DcMotorEx.class);
            when(motor.getCurrentPosition()).thenAnswer(invocation -> ticks.get());
            when(motor.getDirection()).thenReturn(DcMotorSimple.Direction.FORWARD);
            doAnswer(invocation -> {
                ticks.set(0);
                return null;
            }).when(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return motor;
        }
    }
}
