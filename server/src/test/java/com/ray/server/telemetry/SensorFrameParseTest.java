package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;
import org.junit.jupiter.api.Test;
import tools.jackson.databind.ObjectMapper;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * 펌웨어 build_json 이 만드는 JSON 이 SensorFrame 으로 파싱되는지 고정.
 * 깨지면 서버가 프레임을 조용히 버리고 대시보드만 멈춰 추적이 어렵다.
 */
class SensorFrameParseTest {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    /** telemetry.c 의 snprintf 포맷을 그대로 옮긴 샘플. */
    static final String ESP_JSON = """
            {"ts":123456,"sensors":[\
            {"ch":0,"ax":0.012,"ay":-0.998,"az":0.031,"gx":1.20,"gy":-0.30,"gz":0.05,"fault":"none"},\
            {"ch":1,"ax":0.010,"ay":-1.001,"az":0.028,"gx":1.10,"gy":-0.25,"gz":0.02,"fault":"none"},\
            {"ch":6,"ax":0.009,"ay":-0.995,"az":0.033,"gx":1.15,"gy":-0.28,"gz":0.04,"fault":"none"}],\
            "voting":{"result":"ok","used":[0,1,6],"rejected":[]},\
            "balance":{"angle":-1.24,"rate":3.50,"setpoint":-0.80,"uq":0.420},\
            "encoder":{"angle":137.65}}""";

    @Test
    void espJsonParsesIntoSensorFrame() {
        SensorFrame frame = MAPPER.readValue(ESP_JSON, SensorFrame.class);

        assertEquals(123456L, frame.ts());
        assertEquals(3, frame.sensors().size());
        assertEquals(6, frame.sensors().get(2).ch());
        assertEquals("none", frame.sensors().get(0).fault());
        assertEquals(-0.998, frame.sensors().get(0).ay(), 1e-9);

        assertEquals("ok", frame.voting().result());
        assertEquals(3, frame.voting().used().size());
        assertEquals(0, frame.voting().rejected().size());

        assertEquals(-1.24, frame.balance().angle(), 1e-9);
        assertEquals(0.420, frame.balance().uq(), 1e-9);
        assertEquals(137.65, frame.encoder().angle(), 1e-9);
    }
}
