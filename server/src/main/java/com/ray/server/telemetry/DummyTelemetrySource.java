package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.util.List;


// 테스트용 더미 데이터
@Component
@ConditionalOnProperty(name = "telemetry.mock.enabled", havingValue = "true", matchIfMissing = true)
public class DummyTelemetrySource {

    private static final int[] CH = {0, 1, 6};

    private final TelemetrySink sink;
    private double v = 0.0;
    private int dir = 1;

    public DummyTelemetrySource(TelemetrySink sink) {
        this.sink = sink;
    }

    @Scheduled(fixedRate = 50)
    public void tick() {
        v += dir * 0.2;
        if (v >= 10 || v <= -10) dir = -dir;

        var sensors = List.of(mpu(CH[0]), mpu(CH[1]), mpu(CH[2]));
        var allOk = new SensorFrame.Voting.Vote("ok", List.of(CH[0], CH[1], CH[2]), List.of());
        var voting = new SensorFrame.Voting(allOk, allOk);
        var balance = new SensorFrame.Balance(v, v, 0.0, v);
        sink.publish(new SensorFrame(System.currentTimeMillis(),
                sensors, voting, balance, new SensorFrame.Encoder(v)));
    }

    private SensorFrame.Sensor mpu(int ch) {
        return new SensorFrame.Sensor(ch, v, v, v, v, v, v, "none");
    }
}
