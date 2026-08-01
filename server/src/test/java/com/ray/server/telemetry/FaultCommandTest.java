package com.ray.server.telemetry;

import com.ray.server.dto.FaultCommand;
import org.junit.jupiter.api.Test;
import tools.jackson.databind.ObjectMapper;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 대시보드가 보내고 ESP32 가 파싱하는 명령의 모양을 고정.
 * 펌웨어 telemetry.c 의 apply_command 가 ch / mode / rate 를 읽는다.
 */
class FaultCommandTest {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    @Test
    void dashboardJsonParses() {
        FaultCommand cmd = MAPPER.readValue(
                """
                {"cmd":"fault","ch":6,"mode":"drift","rate":0.5}""", FaultCommand.class);

        assertTrue(cmd.isFault());
        assertEquals(6, cmd.ch());
        assertEquals("drift", cmd.mode());
        assertEquals(0.5, cmd.rate(), 1e-9);
    }

    /** 펌웨어로 내려가는 직렬화 결과에 세 필드가 다 있어야 한다. */
    @Test
    void serializesFieldsFirmwareReads() {
        String json = MAPPER.writeValueAsString(
                new FaultCommand("fault", 7, "freeze", 0.0));

        assertTrue(json.contains("\"ch\":7"), json);
        assertTrue(json.contains("\"mode\":\"freeze\""), json);
        assertTrue(json.contains("\"rate\":"), json);
    }

    /** 결함 명령이 아닌 것은 흘려보내지 않는다. */
    @Test
    void otherCommandsAreNotFaults() {
        FaultCommand cmd = MAPPER.readValue(
                """
                {"cmd":"ping","ch":0,"mode":"none","rate":0}""", FaultCommand.class);

        assertFalse(cmd.isFault());
    }
}
