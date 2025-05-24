import time
import network
from machine import SPI, Pin

# WiFi 설정
WIFI_SSID = "YOUR_WIFI_NAME"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

# 재밍 감지 설정
JAMMING_THRESHOLD_KHZ = 10.0  # 10kHz
RSSI_THRESHOLD = -60  # dBm
SCAN_FREQUENCIES = [433.92, 868.3, 915.0]  # MHz

class CC1101:
    """CC1101 RF 모듈 드라이버 (내장)"""
    
    def __init__(self, spi_id=0, cs_pin=5, gdo0_pin=8, gdo2_pin=9):
        self.spi = SPI(spi_id, baudrate=1000000, polarity=0, phase=0,
                      sck=Pin(6), mosi=Pin(7), miso=Pin(4))
        self.cs = Pin(cs_pin, Pin.OUT)
        self.gdo0 = Pin(gdo0_pin, Pin.IN)
        self.gdo2 = Pin(gdo2_pin, Pin.IN)
        
        self.cs.on()  # CS를 HIGH로 초기화
        time.sleep_ms(100)
        
        self.reset()
        self.init_registers()
    
    def reset(self):
        """CC1101 리셋"""
        self.cs.off()
        time.sleep_us(10)
        self.cs.on()
        time.sleep_us(40)
        self.cs.off()
        
        # SRES 커맨드 전송
        self.spi.write(bytearray([0x30]))
        time.sleep_ms(100)
        self.cs.on()
    
    def write_register(self, addr, value):
        """레지스터 쓰기"""
        self.cs.off()
        self.spi.write(bytearray([addr, value]))
        self.cs.on()
    
    def read_register(self, addr):
        """레지스터 읽기"""
        self.cs.off()
        self.spi.write(bytearray([addr | 0x80]))  # 읽기 플래그
        result = self.spi.read(1)
        self.cs.on()
        return result[0]
    
    def read_rssi(self):
        """RSSI 값 읽기"""
        rssi_raw = self.read_register(0x34)  # RSSI 레지스터
        if rssi_raw >= 128:
            rssi_dbm = (rssi_raw - 256) / 2 - 74
        else:
            rssi_dbm = rssi_raw / 2 - 74
        return rssi_dbm
    
    def init_registers(self):
        """CC1101 초기 설정"""
        # 기본 설정 값들
        config = [
            (0x00, 0x29),  # IOCFG2
            (0x01, 0x2E),  # IOCFG1  
            (0x02, 0x06),  # IOCFG0
            (0x03, 0x07),  # FIFOTHR
            (0x04, 0xD3),  # SYNC1
            (0x05, 0x91),  # SYNC0
            (0x06, 0xFF),  # PKTLEN
            (0x07, 0x04),  # PKTCTRL1
            (0x08, 0x45),  # PKTCTRL0
            (0x09, 0x00),  # ADDR
            (0x0A, 0x00),  # CHANNR
            (0x0B, 0x08),  # FSCTRL1
            (0x0C, 0x00),  # FSCTRL0
            # 433MHz 설정
            (0x0D, 0x10),  # FREQ2
            (0x0E, 0xA7),  # FREQ1  
            (0x0F, 0x62),  # FREQ0
            (0x10, 0x5B),  # MDMCFG4
            (0x11, 0xF8),  # MDMCFG3
            (0x12, 0x03),  # MDMCFG2
            (0x13, 0x22),  # MDMCFG1
            (0x14, 0xF8),  # MDMCFG0
        ]
        
        for addr, value in config:
            self.write_register(addr, value)
            time.sleep_ms(1)
    
    def set_frequency(self, freq_mhz):
        """주파수 설정 (MHz 단위)"""
        # 26MHz 크리스탈 기준 계산
        freq_word = int((freq_mhz * 65536) / 26)
        
        freq2 = (freq_word >> 16) & 0xFF
        freq1 = (freq_word >> 8) & 0xFF
        freq0 = freq_word & 0xFF
        
        self.write_register(0x0D, freq2)  # FREQ2
        self.write_register(0x0E, freq1)  # FREQ1
        self.write_register(0x0F, freq0)  # FREQ0
    
    def start_rx(self):
        """수신 모드 시작"""
        self.cs.off()
        self.spi.write(bytearray([0x34]))  # SRX 커맨드
        self.cs.on()
    
    def get_signal_strength(self):
        """신호 강도를 0-4095 범위로 변환"""
        rssi_dbm = self.read_rssi()
        # -110dBm ~ -30dBm 범위를 0-4095로 스케일링
        signal_strength = max(0, min(4095, int((rssi_dbm + 110) * 4095 / 80)))
        return signal_strength, rssi_dbm


class CC1101JammingDetector:
    """재밍 감지 메인 클래스"""
    
    def __init__(self):
        # CC1101 RF 모듈 초기화
        self.rf_module = CC1101(spi_id=0, cs_pin=5, gdo0_pin=8, gdo2_pin=9)
        
        # 상태 변수들
        self.is_monitoring = False
        self.jamming_count = 0
        self.current_frequency = 0.0
        self.current_rssi = -100.0
        self.current_signal_strength = 0
        self.scan_index = 0
        
    def connect_wifi(self):
        """WiFi 연결"""
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        print("📶 WiFi 연결 중...")
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            print(".", end="")
            time.sleep(1)
            timeout -= 1
            
        print()  # 줄바꿈
        if wlan.isconnected():
            print(f"✅ WiFi 연결 성공: {wlan.ifconfig()[0]}")
            return True
        else:
            print("❌ WiFi 연결 실패")
            return False
    
    def scan_frequencies(self):
        """다중 주파수 스캔"""
        max_signal = 0
        detected_freq = 0
        max_rssi = -120
        
        print("📡 주파수 스캔 중...", end=" ")
        
        for freq_mhz in SCAN_FREQUENCIES:
            # 주파수 설정
            self.rf_module.set_frequency(freq_mhz)
            time.sleep_ms(50)  # 안정화 대기
            
            # 수신 모드 시작
            self.rf_module.start_rx()
            time.sleep_ms(100)
            
            # 신호 강도 측정
            signal_strength, rssi_dbm = self.rf_module.get_signal_strength()
            
            print(f"[{freq_mhz}MHz: {rssi_dbm:.1f}dBm]", end=" ")
            
            if signal_strength > max_signal:
                max_signal = signal_strength
                detected_freq = freq_mhz
                max_rssi = rssi_dbm
        
        print()  # 줄바꿈
        return max_signal, detected_freq, max_rssi
    
    def read_sensors(self):
        """센서 데이터 읽기"""
        try:
            # RF 스캔
            self.current_signal_strength, detected_freq, self.current_rssi = self.scan_frequencies()
            
            # 주파수를 kHz로 변환 (시뮬레이션)
            # 실제로는 더 복잡한 신호 분석 필요
            if self.current_rssi > RSSI_THRESHOLD:
                # 강한 신호 감지 시 고주파로 가정
                self.current_frequency = 8.0 + (self.current_rssi + 80) * 0.5
            else:
                self.current_frequency = 8.0 + (self.current_signal_strength / 4095) * 4.0
                
            return True
        except Exception as e:
            print(f"❌ 센서 읽기 오류: {e}")
            return False
    
    def detect_jamming(self):
        """재밍 감지 로직"""
        jamming_detected = False
        
        # 주파수 기준 재밍 감지
        if self.current_frequency >= JAMMING_THRESHOLD_KHZ:
            jamming_detected = True
            self.jamming_count += 1
            print(f"🚨 주파수 재밍 감지! {self.current_frequency:.1f}kHz (임계값: {JAMMING_THRESHOLD_KHZ}kHz)")
        
        # RSSI 기준 추가 검증
        if self.current_rssi > RSSI_THRESHOLD:
            jamming_detected = True
            print(f"🚨 RSSI 재밍 감지! {self.current_rssi:.1f}dBm (임계값: {RSSI_THRESHOLD}dBm)")
        
        return jamming_detected
    
    def send_data_to_dashboard(self):
        """대시보드로 데이터 전송"""
        try:
            # 정확한 신호 강도 계산
            adc_equivalent = int(self.current_signal_strength)
            
            # 대시보드 형식으로 데이터 출력
            data_string = f"FREQ:{self.current_frequency:.1f},ADC:{adc_equivalent},RSSI:{self.current_rssi:.1f},JAM:{self.jamming_count}"
            print(f"📊 {data_string}")
            
        except Exception as e:
            print(f"❌ 데이터 전송 오류: {e}")
    
    def display_status(self, jamming_detected):
        """상태 표시"""
        timestamp = time.localtime()
        time_str = f"{timestamp[3]:02d}:{timestamp[4]:02d}:{timestamp[5]:02d}"
        
        if jamming_detected:
            print(f"🚨 [{time_str}] 재밍 공격 감지됨! (총 {self.jamming_count}회)")
            print(f"   📈 주파수: {self.current_frequency:.1f}kHz")
            print(f"   📡 RSSI: {self.current_rssi:.1f}dBm")
            print(f"   💪 신호강도: {self.current_signal_strength}/4095")
        else:
            print(f"✅ [{time_str}] 정상 상태")
            print(f"   📊 주파수: {self.current_frequency:.1f}kHz, RSSI: {self.current_rssi:.1f}dBm")
    
    def start_monitoring(self):
        """모니터링 시작"""
        self.is_monitoring = True
        print("🔍 CC1101 재밍 감지 모니터링 시작")
        print(f"⚙️  임계값: 주파수 {JAMMING_THRESHOLD_KHZ}kHz, RSSI {RSSI_THRESHOLD}dBm")
        print("=" * 60)
        
        # RF 모듈 초기화
        self.rf_module.start_rx()
        
        while self.is_monitoring:
            try:
                # 센서 데이터 읽기
                if self.read_sensors():
                    # 재밍 감지
                    jamming_detected = self.detect_jamming()
                    
                    # 상태 표시
                    self.display_status(jamming_detected)
                    
                    # 대시보드로 데이터 전송
                    self.send_data_to_dashboard()
                    
                    print("-" * 40)
                
                time.sleep(3)  # 3초마다 측정
                
            except KeyboardInterrupt:
                print("\n🛑 사용자에 의해 모니터링 중단")
                self.is_monitoring = False
                break
            except Exception as e:
                print(f"❌ 모니터링 오류: {e}")
                time.sleep(1)

def main():
    """메인 함수"""
    print("🛡️ CC1101 재밍 감지 시스템 시작")
    print("📡 통합 버전 - RF 전용 (LED, 부저, 온습도 센서 제거)")
    print("=" * 60)
    
    # 시스템 초기화
    detector = CC1101JammingDetector()
    
    # RF 모듈 테스트
    print("🔧 CC1101 RF 모듈 테스트 중...")
    try:
        signal_strength, detected_freq, rssi = detector.scan_frequencies()
        print(f"✅ RF 모듈 정상 작동!")
        print(f"   🎯 최대 신호강도: {signal_strength}/4095")
        print(f"   📡 최대 RSSI: {rssi:.1f}dBm @ {detected_freq}MHz")
    except Exception as e:
        print(f"❌ RF 모듈 오류: {e}")
        print("🔧 연결을 확인하고 다시 시도하세요.")
        return
    
    print("-" * 60)
    
    # WiFi 연결 (선택사항)
    wifi_connected = detector.connect_wifi()
    if not wifi_connected:
        print("⚠️  WiFi 연결 없이 오프라인 모드로 실행")
    
    print("-" * 60)
    
    # 모니터링 시작
    try:
        detector.start_monitoring()
    except Exception as e:
        print(f"❌ 시스템 오류: {e}")
    finally:
        print("🔚 시스템 종료")

# 프로그램 시작점
if __name__ == "__main__":
    main()
