package com.ray.server.dto;

import java.util.List;

/** 로봇 → 서버 → 브라우저로 흐르는 텔레메트리 1프레임 (데이터 계약). */
public record SensorFrame(
        long ts,
        List<Sensor> sensors,
        Voting voting,
        Balance balance,
        Encoder encoder
) {
    public record Sensor(
            int ch,
            double ax, double ay, double az,
            double gx, double gy, double gz,
            String fault   // none | dropout | freeze | drift
    ) {}

    /** 가속도쌍(ax,ay)과 자이로(gz)를 따로 투표한다. */
    public record Voting(Vote accel, Vote gyro) {
        public record Vote(
                String result,          // ok | degraded | fail
                List<Integer> used,
                List<Integer> rejected
        ) {}
    }

    /** main.c 의 angle/rate/setpoint/uq 대응. */
    public record Balance(
            double angle,
            double rate,
            double setpoint,
            double uq
    ) {}

    public record Encoder(double angle) {}
}
