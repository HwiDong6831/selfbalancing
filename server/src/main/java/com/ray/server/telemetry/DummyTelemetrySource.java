package com.ray.server.telemetry;

import com.ray.server.dto.SensorFrame;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.util.ArrayList;
import java.util.List;


// 테스트용 더미 데이터. 이상 이력 표를 ESP 없이 검증하려고 결함을 주기적으로 섞는다.
@Component
@ConditionalOnProperty(name = "telemetry.mock.enabled", havingValue = "true", matchIfMissing = true)
public class DummyTelemetrySource {

    private static final int[] CH = {0, 1, 6};

    // 펌웨어 VOTING_TOL_* 를 물리 단위로 옮긴 값
    private static final double TOL_ACC = 1500 / 16384.0;
    private static final double TOL_GYRO = 300 / 131.0;

    private final TelemetrySink sink;
    private double v = 0.0;
    private int dir = 1;
    private long ticks = 0;

    public DummyTelemetrySource(TelemetrySink sink) {
        this.sink = sink;
    }

    @Scheduled(fixedRate = 20)   // 50Hz. 펌웨어 TELEMETRY_US 와 같다
    public void tick() {
        v += dir * 0.2;
        if (v >= 10 || v <= -10) dir = -dir;

        // 1초 단위로 10초 주기를 돈다
        long phase = (ticks++ / 50) % 10;

        String accelResult = "ok", gyroResult = "ok";
        int badAccel = -1, badGyro = -1;
        switch ((int) phase) {
            case 6 -> { gyroResult  = "degraded"; badGyro  = CH[2]; }
            case 8 -> { accelResult = "degraded"; badAccel = CH[1]; }
            case 9 -> gyroResult = "fail";
            default -> { }
        }

        var sensors = new ArrayList<SensorFrame.Sensor>();
        for (int ch : CH) {
            // 배제된 센서만 임계값을 넘게 띄운다. 표의 오류값·벌어진 폭이 의미를 갖도록.
            double da = (ch == badAccel) ? TOL_ACC * 3 : 0.0;
            double dg = (ch == badGyro) ? TOL_GYRO * 6 : 0.0;
            sensors.add(new SensorFrame.Sensor(ch,
                    v / 100 + da, 1.0 + v / 500, v / 200,
                    v, v, v + dg,
                    -0.004 + ch * 0.002, 0.006 - ch * 0.001, -1.2 + ch * 0.4,
                    "none"));
        }

        var tol = new SensorFrame.Voting.Tol(TOL_ACC, TOL_GYRO);
        var voting = new SensorFrame.Voting(tol,
                vote(accelResult, badAccel, v / 100, 1.0 + v / 500),
                vote(gyroResult, badGyro, v));
        var balance = new SensorFrame.Balance(v, v, 0.0, v);
        sink.publish(new SensorFrame(System.currentTimeMillis(),
                sensors, voting, balance, new SensorFrame.Encoder(v)));
    }

    private SensorFrame.Voting.Vote vote(String result, int badCh, double... val) {
        var used = new ArrayList<Integer>();
        var rejected = new ArrayList<Integer>();
        for (int ch : CH) {
            // fail 은 채택이 없다 — 펌웨어도 used 를 전부 false 로 둔다
            if (result.equals("fail") || ch == badCh) rejected.add(ch);
            else used.add(ch);
        }
        var v = new ArrayList<Double>();
        if (!result.equals("fail")) for (double d : val) v.add(d);
        return new SensorFrame.Voting.Vote(result, used, rejected, v);
    }
}
