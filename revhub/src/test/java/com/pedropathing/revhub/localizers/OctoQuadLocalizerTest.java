/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.revhub.localizers;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad.LocalizerStatus;
import org.junit.Test;

public class OctoQuadLocalizerTest {
    @Test
    public void returnsImmediatelyWhenRunning() {
        OctoQuad device = mock(OctoQuad.class);
        when(device.getLocalizerStatus()).thenReturn(LocalizerStatus.RUNNING);

        OctoQuadLocalizer.awaitRunning(device, 0);

        verify(device).getLocalizerStatus();
    }

    @Test(timeout = 2000)
    public void pollsUntilRunning() {
        OctoQuad device = mock(OctoQuad.class);
        when(device.getLocalizerStatus())
                .thenReturn(LocalizerStatus.WARMING_UP_IMU, LocalizerStatus.CALIBRATING_IMU, LocalizerStatus.RUNNING);

        OctoQuadLocalizer.awaitRunning(device, 1000);

        verify(device, times(3)).getLocalizerStatus();
    }

    @Test(timeout = 2000)
    public void timesOutWithTheLastStatus() {
        OctoQuad device = mock(OctoQuad.class);
        when(device.getLocalizerStatus()).thenReturn(LocalizerStatus.FAULT_NO_IMU);

        IllegalStateException failure = assertThrows(
                IllegalStateException.class, () -> OctoQuadLocalizer.awaitRunning(device, 20));

        assertTrue(failure.getMessage().contains("Timed out"));
        assertTrue(failure.getMessage().contains("FAULT_NO_IMU"));
        assertFalse(Thread.currentThread().isInterrupted());
    }

    @Test
    public void preservesAnExistingInterrupt() {
        OctoQuad device = mock(OctoQuad.class);
        when(device.getLocalizerStatus()).thenReturn(LocalizerStatus.CALIBRATING_IMU);
        Thread.currentThread().interrupt();
        try {
            IllegalStateException failure = assertThrows(
                    IllegalStateException.class, () -> OctoQuadLocalizer.awaitRunning(device, 1000));
            assertTrue(failure.getMessage().contains("Interrupted"));
            assertTrue(failure.getMessage().contains("CALIBRATING_IMU"));
            assertTrue(Thread.currentThread().isInterrupted());
        } finally {
            Thread.interrupted();
        }
    }
}
