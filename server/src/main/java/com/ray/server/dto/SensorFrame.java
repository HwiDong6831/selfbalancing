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
    /** ax/ay/gz 는 영점 뺀 비교값. ax0/ay0/gz0 은 그때 뺀 영점. */
    public record Sensor(
            int ch,
            double ax, double ay, double az,
            double gx, double gy, double gz,
            double ax0, double ay0, double gz0,
            String fault   // none | dropout | freeze | drift
    ) {}

    /** 가속도쌍(ax,ay)과 자이로(gz)를 따로 투표한다. */
    public record Voting(Tol tol, Vote accel, Vote gyro) {
        /** 이상치 임계값. 펌웨어 상수라 프레임마다 같다. */
        public record Tol(double accel, double gyro) {}

        public record Vote(
                String result,          // ok | degraded | fail
                List<Integer> used,
                List<Integer> rejected,
                List<Double> val        // 채택 평균. 자이로는 1개
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
