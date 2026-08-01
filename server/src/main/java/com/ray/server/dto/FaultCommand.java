package com.ray.server.dto;

/**
 * 대시보드 → 서버 → ESP32 로 흐르는 결함 주입 명령.
 *
 * @param mode none | dropout | freeze | drift
 * @param rate drift 전용. 초당 임계값의 몇 배로 벌어지는가
 */
public record FaultCommand(String cmd, int ch, String mode, double rate) {

    public static final String NONE = "none";

    public boolean isFault() {
        return "fault".equals(cmd);
    }
}
