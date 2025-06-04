import time
import network
from machine import SPI, Pin

# WiFi 설정
WIFI_SSID = "Host"
WIFI_PASSWORD = "a01160116!"

# RSSI 기반 재밍 감지 설정
JAMMING_THRESHOLD_RSSI = -50.0  # dBm (이 값보다 높은 신호는 재밍으로 판단)
NORMAL_RSSI_RANGE = (-110, -70)  # 정상 범위 dBm
SCAN_FREQUENCIES = [433.92, 868.3, 915.0]  # MHz

# 적응 모드 임계값 설정
SENSITIVITY_LEVELS = {
    0: -50.0,  # 표준 모드
    1: -53.0,  # 민감 모드  
    2: -55.0   # 고민감 모드
}

class CC1101:
    """CC1101 RF 모듈 드라이버 (RSSI 감지 최적화)"""
    
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
        """RSSI 값 읽기 (재밍 감지용)"""
        # RSSI 레지스터에서 직접 읽기
        rssi_raw = self.read_register(0x34)  # RSSI 레지스터
        
        # CC1101 RSSI 계산 공식
        if rssi_raw >= 128:
            rssi_dbm = (rssi_raw - 256) / 2 - 74
        else:
            rssi_dbm = rssi_raw / 2 - 74
            
        return rssi_dbm
    
    def read_lqi(self):
        """LQI (Link Quality Indicator) 읽기"""
        lqi_raw = self.read_register(0x33)  # LQI 레지스터
        return lqi_raw & 0x7F  # 하위 7비트만 사용
    
    def init_registers(self):
        """CC1101 RSSI 측정 최적화 설정"""
        # RSSI 측정을 위한 최적화된 설정
        config = [
            (0x00, 0x29),  # IOCFG2 - GDO2 설정
            (0x01, 0x2E),  # IOCFG1 - GDO1 설정
            (0x02, 0x06),  # IOCFG0 - GDO0 설정
            (0x03, 0x07),  # FIFOTHR - FIFO 임계값
            (0x04, 0xD3),  # SYNC1 - 동기화 워드
            (0x05, 0x91),  # SYNC0 - 동기화 워드
            (0x06, 0xFF),  # PKTLEN - 패킷 길이
            (0x07, 0x04),  # PKTCTRL1 - 패킷 제어
            (0x08, 0x45),  # PKTCTRL0 - 패킷 제어
            (0x09, 0x00),  # ADDR - 주소
            (0x0A, 0x00),  # CHANNR - 채널 번호
            (0x0B, 0x0C),  # FSCTRL1 - 주파수 제어 (RSSI 측정 최적화)
            (0x0C, 0x00),  # FSCTRL0 - 주파수 제어
            # 433MHz 기본 설정
            (0x0D, 0x10),  # FREQ2
            (0x0E, 0xA7),  # FREQ1  
            (0x0F, 0x62),  # FREQ0
            # RSSI 측정을 위한 모뎀 설정
            (0x10, 0x5B),  # MDMCFG4 - 모뎀 설정
            (0x11, 0xF8),  # MDMCFG3 - 모뎀 설정
            (0x12, 0x03),  # MDMCFG2 - 모뎀 설정
            (0x13, 0x22),  # MDMCFG1 - 모뎀 설정
            (0x14, 0xF8),  # MDMCFG0 - 모뎀 설정
            # AGC 제어 (RSSI 안정성 향상)
            (0x15, 0x62),  # DEVIATN - 편차 설정
            (0x16, 0x07),  # MCSM2 - 메인 상태 머신 제어
            (0x17, 0x30),  # MCSM1 - 메인 상태 머신 제어
            (0x18, 0x18),  # MCSM0 - 메인 상태 머신 제어
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
        
        # 주파수 변경 후 안정화 대기
        time.sleep_ms(10)
    
    def start_rx(self):
        """수신 모드 시작"""
        self.cs.off()
        self.spi.write(bytearray([0x34]))  # SRX 커맨드
        self.cs.on()
        time.sleep_ms(5)  # 수신 모드 안정화
    
    def calibrate(self):
        """수동 보정 (RSSI 정확도 향상)"""
        self.cs.off()
        self.spi.write(bytearray([0x33]))  # SCAL 커맨드
        self.cs.on()
        time.sleep_ms(50)  # 보정 완료 대기
    
    def get_signal_strength_adc(self):
        """신호 강도를 ADC 형태로 변환 (대시보드 호환성)"""
        rssi_dbm = self.read_rssi()
        # -110dBm ~ -30dBm 범위를 0-4095로 스케일링
        signal_strength = max(0, min(4095, int((rssi_dbm + 110) * 4095 / 80)))
        return signal_strength, rssi_dbm


class CC1101JammingDetector:
    """RSSI 기반 재밍 감지 메인 클래스"""
    
    def __init__(self):
        # CC1101 RF 모듈 초기화
        self.rf_module = CC1101(spi_id=0, cs_pin=5, gdo0_pin=8, gdo2_pin=9)
        
        # 상태 변수들
        self.is_monitoring = False
        self.jamming_count = 0
        self.current_frequency = SCAN_FREQUENCIES[0]
        self.current_rssi = -80.0
        self.current_signal_strength = 0
        self.scan_index = 0
        self.adaptation_mode = 0
        
        # RSSI 이력 (변화 패턴 분석용)
        self.rssi_history = []
        self.max_history_length = 10
        
        # 재밍 감지 관련
        self.consecutive_jamming = 0
        self.jamming_detected = False
        
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
            
        print()
        if wlan.isconnected():
            print(f"✅ WiFi 연결 성공: {wlan.ifconfig()[0]}")
            return True
        else:
            print("❌ WiFi 연결 실패")
            return False
    
    def scan_frequencies_for_rssi(self):
        """주파수별 RSSI 스캔 및 최강 신호 감지"""
        max_rssi = -120.0
        detected_freq = SCAN_FREQUENCIES[0]
        max_signal_strength = 0
        
        print("📡 RSSI 스캔 중...", end=" ")
        
        for freq_mhz in SCAN_FREQUENCIES:
            # 주파수 설정
            self.rf_module.set_frequency(freq_mhz)
            
            # 수신 모드 시작
            self.rf_module.start_rx()
            time.sleep_ms(100)  # 안정화 대기
            
            # 여러 번 측정하여 평균값 계산 (노이즈 감소)
            rssi_readings = []
            for _ in range(5):
                rssi = self.rf_module.read_rssi()
                rssi_readings.append(rssi)
                time.sleep_ms(20)
            
            avg_rssi = sum(rssi_readings) / len(rssi_readings)
            signal_strength, _ = self.rf_module.get_signal_strength_adc()
            
            print(f"[{freq_mhz}MHz: {avg_rssi:.1f}dBm]", end=" ")
            
            # 가장 강한 신호 기록 (RSSI가 높을수록 강한 신호)
            if avg_rssi > max_rssi:
                max_rssi = avg_rssi
                detected_freq = freq_mhz
                max_signal_strength = signal_strength
        
        print()
        return max_signal_strength, detected_freq, max_rssi
    
    def update_rssi_history(self, rssi):
        """RSSI 이력 업데이트 (패턴 분석용)"""
        self.rssi_history.append(rssi)
        if len(self.rssi_history) > self.max_history_length:
            self.rssi_history.pop(0)
    
    def analyze_rssi_pattern(self):
        """RSSI 패턴 분석 (급격한 변화 감지)"""
        if len(self.rssi_history) < 3:
            return False
        
        # 최근 3개 값의 평균과 이전 값들 비교
        recent_avg = sum(self.rssi_history[-3:]) / 3
        older_avg = sum(self.rssi_history[:-3]) / len(self.rssi_history[:-3])
        
        # 급격한 RSSI 증가 (10dBm 이상) 감지
        if recent_avg - older_avg > 10:
            print(f"📈 급격한 RSSI 증가 감지: {older_avg:.1f} → {recent_avg:.1f} dBm")
            return True
        
        return False
    
    def get_current_threshold(self):
        """현재 적응 모드에 따른 임계값 반환"""
        return SENSITIVITY_LEVELS.get(self.adaptation_mode, JAMMING_THRESHOLD_RSSI)
    
    def read_sensors(self):
        """센서 데이터 읽기 (RSSI 중심)"""
        try:
            # RF 보정 (주기적으로)
            if self.scan_index % 10 == 0:
                self.rf_module.calibrate()
            
            # 주파수별 RSSI 스캔
            self.current_signal_strength, self.current_frequency, self.current_rssi = self.scan_frequencies_for_rssi()
            
            # RSSI 이력 업데이트
            self.update_rssi_history(self.current_rssi)
            
            # 스캔 인덱스 업데이트
            self.scan_index = (self.scan_index + 1) % len(SCAN_FREQUENCIES)
            
            return True
            
        except Exception as e:
            print(f"❌ 센서 읽기 오류: {e}")
            return False
    
    def detect_jamming(self):
        """RSSI 기반 재밍 감지 로직"""
        jamming_detected = False
        current_threshold = self.get_current_threshold()
        
        # 1. RSSI 임계값 기반 감지
        if self.current_rssi >= current_threshold:
            jamming_detected = True
            self.consecutive_jamming += 1
            print(f"🚨 RSSI 재밍 감지! {self.current_rssi:.1f}dBm >= {current_threshold}dBm")
        else:
            self.consecutive_jamming = 0
        
        # 2. RSSI 패턴 변화 기반 감지
        if self.analyze_rssi_pattern():
            jamming_detected = True
            print(f"🚨 RSSI 패턴 변화 재밍 감지!")
        
        # 3. 연속 감지 시 카운트 증가
        if jamming_detected and not self.jamming_detected:
            self.jamming_count += 1
            self.jamming_detected = True
            print(f"🔥 재밍 공격 #{self.jamming_count} 감지됨!")
        elif not jamming_detected:
            self.jamming_detected = False
        
        # 4. 적응 모드 업데이트
        self.update_adaptation_mode()
        
        return jamming_detected
    
    def update_adaptation_mode(self):
        """적응 모드 업데이트 (재밍 횟수에 따라)"""
        if self.jamming_count >= 6:
            self.adaptation_mode = 2  # 고민감 모드
        elif self.jamming_count >= 3:
            self.adaptation_mode = 1  # 민감 모드
        else:
            self.adaptation_mode = 0  # 표준 모드
    
    def send_data_to_dashboard(self):
        """대시보드로 RSSI 중심 데이터 전송"""
        try:
            # 대시보드가 파싱할 수 있는 형식으로 출력 (RSSI 우선)
            data_string = f"RSSI:{self.current_rssi:.1f},FREQ:{self.current_frequency:.2f},ADC:{self.current_signal_strength},JAM:{self.jamming_count}"
            print(data_string)
            
            # 추가 정보 (별도 라인)
            print(f"📊 RSSI 데이터 전송: {self.current_rssi:.1f}dBm @ {self.current_frequency}MHz")
            
        except Exception as e:
            print(f"❌ 데이터 전송 오류: {e}")
    
    def display_status(self, jamming_detected):
        """상태 표시 (RSSI 중심)"""
        timestamp = time.localtime()
        time_str = f"{timestamp[3]:02d}:{timestamp[4]:02d}:{timestamp[5]:02d}"
        current_threshold = self.get_current_threshold()
        
        if jamming_detected:
            print(f"🚨 [{time_str}] RSSI 재밍 공격 감지됨! (총 {self.jamming_count}회)")
            print(f"   📈 RSSI: {self.current_rssi:.1f}dBm (임계값: {current_threshold}dBm)")
            print(f"   📡 주파수: {self.current_frequency:.2f}MHz")
            print(f"   💪 신호강도: {self.current_signal_strength}/4095")
            print(f"   🔧 적응모드: {self.adaptation_mode} (연속감지: {self.consecutive_jamming})")
        else:
            print(f"✅ [{time_str}] 정상 상태")
            print(f"   📊 RSSI: {self.current_rssi:.1f}dBm @ {self.current_frequency:.2f}MHz")
            print(f"   🎯 임계값: {current_threshold}dBm (모드: {self.adaptation_mode})")
            
        # RSSI 히스토리 표시 (최근 5개)
        if len(self.rssi_history) >= 5:
            recent_rssi = self.rssi_history[-5:]
            print(f"   📈 RSSI 히스토리: {[f'{r:.1f}' for r in recent_rssi]}")
    
    def start_monitoring(self):
        """RSSI 기반 모니터링 시작"""
        self.is_monitoring = True
        print("🔍 CC1101 RSSI 재밍 감지 모니터링 시작")
        print(f"⚙️  기본 임계값: {JAMMING_THRESHOLD_RSSI}dBm")
        print(f"📡 스캔 주파수: {SCAN_FREQUENCIES} MHz")
        print("=" * 60)
        
        # 시작 신호 전송
        print("STATUS:RSSI_MONITORING_STARTED")
        
        # RF 모듈 초기 보정
        try:
            print("🔧 RF 모듈 보정 중...")
            self.rf_module.calibrate()
            self.rf_module.start_rx()
            time.sleep_ms(100)
            print("✅ RF 모듈 준비 완료")
        except Exception as e:
            print(f"ERROR:RF_INIT_FAILED:{e}")
            return
        
        while self.is_monitoring:
            try:
                # 주기적 하트비트
                print("HEARTBEAT:RSSI_MONITORING")
                
                # RSSI 센서 데이터 읽기
                if self.read_sensors():
                    # RSSI 기반 재밍 감지
                    jamming_detected = self.detect_jamming()
                    
                    # 상태 표시
                    self.display_status(jamming_detected)
                    
                    # 대시보드로 데이터 전송
                    self.send_data_to_dashboard()
                    
                    print("-" * 50)
                else:
                    print("ERROR:RSSI_SENSOR_READ_FAILED")
                
                time.sleep(2)  # 2초마다 RSSI 측정
                
            except KeyboardInterrupt:
                print("\n🛑 사용자에 의해 RSSI 모니터링 중단")
                print("STATUS:RSSI_MONITORING_STOPPED")
                self.is_monitoring = False
                break
            except Exception as e:
                print(f"ERROR:RSSI_MONITORING_EXCEPTION:{e}")
                time.sleep(1)

def main():
    """메인 함수"""
    print("STATUS:SYSTEM_STARTING")
    print("🛡️ CC1101 RSSI 재밍 감지 시스템 시작")
    print("📡 RSSI 기반 재밍 감지 - 신호 강도 모니터링")
    print(f"🎯 RSSI 재밍 임계값: {JAMMING_THRESHOLD_RSSI}dBm")
    print(f"📊 적응 임계값: {list(SENSITIVITY_LEVELS.values())} dBm")
    print("=" * 60)
    
    try:
        # 시스템 초기화
        detector = CC1101JammingDetector()
        print("STATUS:RSSI_DETECTOR_CREATED")
        
        # RF 모듈 RSSI 테스트
        print("🔧 CC1101 RF 모듈 RSSI 테스트 중...")
        try:
            signal_strength, detected_freq, rssi = detector.scan_frequencies_for_rssi()
            print(f"✅ RF 모듈 RSSI 기능 정상!")
            print(f"   🎯 최대 RSSI: {rssi:.1f}dBm @ {detected_freq}MHz")
            print(f"   📊 신호강도: {signal_strength}/4095")
            
            if rssi >= JAMMING_THRESHOLD_RSSI:
                print(f"⚠️  현재 RSSI가 임계값({JAMMING_THRESHOLD_RSSI}dBm)을 초과합니다!")
            
            print("STATUS:RSSI_MODULE_OK")
        except Exception as e:
            print(f"ERROR:RSSI_MODULE_FAILED:{e}")
            print("🔧 연결을 확인하고 다시 시도하세요.")
        
        print("-" * 60)
        
        # WiFi 연결 (선택사항)
        try:
            wifi_connected = detector.connect_wifi()
            if not wifi_connected:
                print("⚠️  WiFi 연결 없이 오프라인 RSSI 모니터링 실행")
            print("STATUS:WIFI_CHECKED")
        except Exception as e:
            print(f"ERROR:WIFI_FAILED:{e}")
            print("STATUS:WIFI_SKIPPED")
        
        print("-" * 60)
        
        # RSSI 모니터링 시작
        print("STATUS:STARTING_RSSI_MONITORING")
        detector.start_monitoring()
        
    except Exception as e:
        print(f"ERROR:MAIN_EXCEPTION:{e}")
        print("STATUS:SYSTEM_ERROR")
    finally:
        print("STATUS:SYSTEM_SHUTDOWN")
        print("🔚 RSSI 재밍 감지 시스템 종료")

# 프로그램 시작점
if __name__ == "__main__":
    try:
        print("STATUS:PROGRAM_START")
        print("🚀 RSSI 기반 재밍 감지 프로그램 시작")
        time.sleep(1)
        main()
    except Exception as e:
        print(f"ERROR:STARTUP_FAILED:{e}")
        print("STATUS:STARTUP_ERROR")

# 자동 실행
print("STATUS:AUTO_START_TRIGGERED")
main()
