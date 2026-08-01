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
            {"ch":0,"ax":0.0120,"ay":-0.9980,"az":0.031,"gx":1.20,"gy":-0.30,"gz":0.05,\
            "ax0":-0.0043,"ay0":0.0061,"gz0":-1.20,"fault":"none"},\
            {"ch":1,"ax":0.0100,"ay":-1.0010,"az":0.028,"gx":1.10,"gy":-0.25,"gz":0.02,\
            "ax0":0.0012,"ay0":-0.0033,"gz0":0.84,"fault":"none"},\
            {"ch":6,"ax":0.0090,"ay":-0.9950,"az":0.033,"gx":1.15,"gy":-0.28,"gz":2.94,\
            "ax0":0.0031,"ay0":-0.0028,"gz0":0.36,"fault":"none"}],\
            "voting":{"tol":{"accel":0.0916,"gyro":2.29},\
            "accel":{"result":"ok","used":[0,1,6],"rejected":[],"val":[0.0103,-0.9980]},\
            "gyro":{"result":"degraded","used":[0,1],"rejected":[6],"val":[0.03]}},\
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
        assertEquals(-1.20, frame.sensors().get(0).gz0(), 1e-9);

        // 임계값은 펌웨어 상수를 물리 단위로 옮긴 값이다.
        assertEquals(0.0916, frame.voting().tol().accel(), 1e-9);
        assertEquals(2.29, frame.voting().tol().gyro(), 1e-9);

        // 가속도는 셋 다 채택, 자이로만 하나 배제된 상태 — 신호별로 따로 판정된다.
        assertEquals("ok", frame.voting().accel().result());
        assertEquals(3, frame.voting().accel().used().size());
        assertEquals(0, frame.voting().accel().rejected().size());
        assertEquals(2, frame.voting().accel().val().size());
        assertEquals(-0.9980, frame.voting().accel().val().get(1), 1e-9);

        assertEquals("degraded", frame.voting().gyro().result());
        assertEquals(2, frame.voting().gyro().used().size());
        assertEquals(6, frame.voting().gyro().rejected().get(0));
        assertEquals(1, frame.voting().gyro().val().size());

        assertEquals(-1.24, frame.balance().angle(), 1e-9);
        assertEquals(0.420, frame.balance().uq(), 1e-9);
        assertEquals(137.65, frame.encoder().angle(), 1e-9);
    }
}
