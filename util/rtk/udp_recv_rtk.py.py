import socket
import time

BOARD_IP = "192.168.0.1"
BOARD_PORT = 5555

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0)

# UDP client처럼 보드에 연결
sock.connect((BOARD_IP, BOARD_PORT))

# C099-F9P 쪽에 "active UDP client"를 알려주기 위한 빈 패킷
# 안 되면 아래 b""를 b"\x00"으로 바꿔보면 됨
sock.send(b"")

print(f"Connected to UDP {BOARD_IP}:{BOARD_PORT}")
print("Waiting for RTK/GNSS data...")

with open("rtk_udp_log.bin", "ab") as f:
    while True:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            print("[timeout] no data")
            # UDP 연결 유지를 위해 가끔 빈 패킷 전송
            sock.send(b"")
            continue

        recv_time = time.time()
        f.write(data)
        f.flush()

        # 앞부분만 확인용 출력
        print(f"[{recv_time:.3f}] {len(data)} bytes")

        # NMEA 문자열이면 사람이 읽을 수 있게 출력
        try:
            text = data.decode("ascii", errors="ignore")
            if "$" in text:
                print(text.strip())
            else:
                print(data[:32].hex())
        except Exception:
            print(data[:32].hex())