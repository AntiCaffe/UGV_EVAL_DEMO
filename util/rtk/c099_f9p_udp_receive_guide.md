# C099-F9P RTK UDP 수신 가이드

## 1. 작동 방식

C099-F9P RTK 보드를 **Wi-Fi AP 모드**로 설정하면, 보드가 직접 Wi-Fi 신호를 생성합니다.  
Ubuntu 노트북은 해당 Wi-Fi에 접속한 뒤, UDP를 통해 RTK/GNSS 데이터를 수신합니다.

```text
C099-F9P RTK Board
  └─ Wi-Fi AP / UDP Server
        ▲
        │ Wi-Fi
        ▼
Ubuntu Laptop
  └─ UDP Client Python Code
```

기본 접속 정보는 다음과 같습니다.

```text
SSID        : C099-F9P
Password    : 123456789
Board IP    : 192.168.0.1
UDP Port    : 5555
```

---

## 2. RTK 보드 설정

ODIN-W2 CLI에 접속한 뒤, 보드를 **Wi-Fi AP + Rover 모드**로 설정합니다.

```text
/mem_store/run wifi_ap
/mem_store/run rover
```

설정 후 보드를 재부팅합니다.

Ubuntu에서 Wi-Fi 목록을 확인합니다.

```bash
nmcli dev wifi rescan
nmcli dev wifi list | grep C099
```

보드 Wi-Fi에 접속합니다.

```bash
nmcli dev wifi connect "C099-F9P" password "123456789"
```

연결 확인:

```bash
ping 192.168.0.1
```

---

## 3. Python UDP 수신 코드

아래 코드를 `udp_recv_rtk.py`로 저장합니다.

```python
import socket
import time

BOARD_IP = "192.168.0.1"
BOARD_PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0)

# C099-F9P UDP server에 연결
sock.connect((BOARD_IP, BOARD_PORT))

# UDP client 등록용 초기 패킷
sock.send(b"")

print(f"Connected to UDP {BOARD_IP}:{BOARD_PORT}")
print("Waiting for RTK/GNSS data...")

with open("rtk_udp_log.bin", "ab") as f:
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            print("[timeout] no data")
            sock.send(b"")
            continue

        recv_time = time.time()

        # raw 데이터 저장
        f.write(data)
        f.flush()

        print(f"[{recv_time:.3f}] {len(data)} bytes")

        # NMEA 문자열이면 보기 좋게 출력
        text = data.decode("ascii", errors="ignore")
        if "$" in text:
            print(text.strip())
        else:
            print(data[:32].hex())
```

실행:

```bash
python3 udp_recv_rtk.py
```

정상적으로 데이터가 들어오면 다음과 같이 출력됩니다.

```text
Connected to UDP 192.168.0.1:5555
Waiting for RTK/GNSS data...
[1720000000.123] 128 bytes
...
```

수신된 raw 데이터는 `rtk_udp_log.bin`에 저장됩니다.

---

## 4. 데이터가 안 들어올 때 확인할 것

### 4.1 Wi-Fi 연결 확인

```bash
ping 192.168.0.1
```

응답이 없으면 노트북이 C099-F9P Wi-Fi에 제대로 연결되지 않은 것입니다.

### 4.2 UDP 패킷 확인

Wi-Fi 인터페이스 이름을 확인합니다.

```bash
iw dev
```

예를 들어 인터페이스가 `wlp3s0`라면:

```bash
sudo tcpdump -i wlp3s0 udp port 5555 -X
```

UDP 패킷이 보이면 네트워크 연결은 정상입니다.

### 4.3 ZED-F9P 출력 메시지 확인

Wi-Fi로 전달되는 데이터는 ZED-F9P에서 ODIN-W2를 거쳐 나옵니다.  
따라서 필요한 GNSS 메시지가 ZED-F9P의 I2C 출력에 enable되어 있어야 합니다.

예시 메시지:

```text
NMEA-GGA
NMEA-RMC
UBX-NAV-PVT
UBX-NAV-RELPOSNED
UBX-RXM-RTCM
```

---

## 5. 주의사항

UDP 패킷이 노트북에 도착한 시간은 정확한 GNSS 측정 시간이 아닙니다.  
Wi-Fi, UDP, OS 네트워크 스택 지연이 포함되므로 LiDAR와 time sync를 맞출 때는 패킷 수신 시간이 아니라 GNSS 메시지 내부 timestamp 또는 PPS/TIMEPULSE 기반 동기화를 사용하는 것이 좋습니다.

---

## 6. ROS2 토픽으로 받기

이 워크스페이스에는 C099-F9P UDP NMEA를 `ublox_gps`가 쓰던 토픽 이름으로 변환하는 브릿지가 추가되어 있습니다.

발행 토픽:

```text
/ublox_gps_node/navpvt        ublox_msgs/msg/NavPVT
/ublox_gps_node/fix           sensor_msgs/msg/NavSatFix
/ublox_gps_node/fix_velocity  geometry_msgs/msg/TwistWithCovarianceStamped
```

실행:

```bash
source install/setup.bash
ros2 run rtk_livox_dataset_tools c099_udp_bridge \
  --host 192.168.0.1 \
  --port 5555 \
  --configure-rate-hz 10 \
  --raw-log rtk_udp_log.bin
```

기존 RTK 수집 launch에서 serial `ublox_gps` 대신 C099 UDP를 쓰려면:

```bash
source install/setup.bash
ros2 launch rtk_livox_dataset_tools rtk_collection.launch.py \
  start_ublox:=false \
  start_c099_udp:=true \
  c099_configure_rate_hz:=10 \
  start_ntrip:=false \
  bag_uri:=bags/run_01_rtk_c099
```

`--configure-rate-hz 10`은 `ublox_gps`의 `rate: 10.0`, `nav_rate: 1`과 같은 의미의 UBX `CFG-RATE` 명령을 UDP로 보냅니다. C099의 UDP 입력이 ZED-F9P로 포워딩되는 설정이면 바로 적용됩니다. 적용되지 않으면 USB/u-center 또는 `ublox_gps` 시리얼 연결로 `rate: 10.0`을 한 번 설정한 뒤 Wi-Fi UDP 수신을 사용하세요.

현재 수신 예시처럼 `$GNGGA,,,,,,0,...`이면 아직 GNSS fix가 없는 상태입니다. 이때 브릿지는 `/ublox_gps_node/fix`를 `NO_FIX`, `/ublox_gps_node/navpvt`를 `fix_type=0`으로 발행합니다. 안테나 위치와 위성 수신 상태가 좋아져서 GGA quality가 `4`이면 RTK fixed, `5`이면 RTK float로 매핑됩니다.

---

## 7. 최소 테스트 코드

수신 여부만 빠르게 확인하고 싶다면 아래 코드만 사용해도 됩니다.

```python
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect(("192.168.0.1", 5555))
sock.send(b"")

while True:
    data = sock.recv(4096)
    print(len(data), data[:16].hex())
```
