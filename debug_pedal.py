import struct
import os
import sys

def debug_pedal():
    # 시도해볼 장치 목록
    devices = [
        "/dev/input/by-id/usb-PCsensor_FootSwitch-event-kbd",
        "/dev/input/by-id/usb-PCsensor_FootSwitch-event-if01",
        "/dev/input/event24"
    ]
    
    device_path = None
    for d in devices:
        if os.path.exists(d):
            device_path = d
            break
            
    if not device_path:
        print("❌ 풋 페달 장치를 찾을 수 없습니다.")
        return

    print(f"🔍 장치 확인됨: {device_path}")
    print("👟 페달을 밟아보세요. (Ctrl+C로 종료)")

    # 64-bit Linux input_event format: timeval(8, 8), type(2), code(2), value(4)
    event_format = "QQHHi"
    event_size = struct.calcsize(event_format)

    try:
        with open(device_path, "rb") as f:
            while True:
                data = f.read(event_size)
                if not data:
                    break
                
                _, _, ev_type, ev_code, ev_value = struct.unpack(event_format, data)
                
                # ev_type 1: EV_KEY
                if ev_type == 1:
                    print(f"🔔 EVENT: Type={ev_type}, Code={ev_code}, Value={ev_value}")
    except PermissionError:
        print("❌ 권한 오류: sudo python3 debug_pedal.py 로 실행해보세요.")
    except Exception as e:
        print(f"❌ 에러 발생: {e}")

if __name__ == "__main__":
    debug_pedal()
