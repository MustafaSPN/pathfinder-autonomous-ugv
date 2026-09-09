import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import serial
import socket
import base64
import threading
import time
import math
import pynmea2
import struct

class rtd100_driver(Node):
    def __init__(self):
        super().__init__('swegeo_driver_node')

        # --- AYARLAR ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps_link')
        
        # NTRIP Ayarları
        self.declare_parameter('ntrip_enable', False)
        self.declare_parameter('ntrip_host', '192.168.1.100')
        self.declare_parameter('ntrip_port', 2101)
        self.declare_parameter('ntrip_mountpoint', 'MOUNTPOINT')
        self.declare_parameter('ntrip_user', 'user')
        self.declare_parameter('ntrip_pass', 'pass')
        self.declare_parameter('message_type', 'ascii')
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.isBinary = self.get_parameter('message_type').value.lower() == 'binary'
        
        # --- DEGISKENLER ---
        # GPS'ten okunan son GGA verisini burada tutacağız (Yöntem 2 için gerekli)
        self.latest_gga = None 
        self.buffer = bytearray() # Binary okuma için tampon

        # --- YAYINCILAR ---
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/gps/imu', 10)

        # --- SERİ PORT ---
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Swegeo RTD100 Baglandi: {self.port}. Binary Mode: {self.isBinary}")
        except Exception as e:
            self.get_logger().error(f"Seri Port Hatasi: {e}")
            return

        # --- NTRIP THREAD ---
        if self.get_parameter('ntrip_enable').value:
            self.stop_ntrip = False
            self.ntrip_thread = threading.Thread(target=self.run_ntrip)
            self.ntrip_thread.daemon = True
            self.ntrip_thread.start()

        # --- OKUMA DÖNGÜSÜ ---
        self.create_timer(0.01, self.read_serial)

    def run_ntrip(self):
        """ NTRIP İstemcisi - Yöntem 2 (Periyodik GGA Gönderimi) """
        host = self.get_parameter('ntrip_host').value
        port = self.get_parameter('ntrip_port').value
        mnt = self.get_parameter('ntrip_mountpoint').value
        user = self.get_parameter('ntrip_user').value
        pasw = self.get_parameter('ntrip_pass').value


        while not self.stop_ntrip and rclpy.ok():
            try:
                self.get_logger().info(f"NTRIP Sunucusuna Baglaniliyor: {host}:{port}")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3) # Timeout'u kısa tuttum ki GGA gönderme kontrolüne sık düşsün
                s.connect((host, port))
                
                auth = base64.b64encode(f"{user}:{pasw}".encode()).decode()
                headers = (f"GET /{mnt} HTTP/1.0\r\n"
                           f"User-Agent: ROS2Driver\r\n"
                           f"Authorization: Basic {auth}\r\n"
                           f"Accept: */*\r\n"
                           f"Connection: close\r\n\r\n")
                
                s.sendall(headers.encode())
                
                # İlk cevap kontrolü
                response = s.recv(4096)
                if b"ICY 200 OK" in response or b"HTTP/1.0 200 OK" in response or b"HTTP/1.1 200 OK" in response:
                    self.get_logger().info("NTRIP Baglandi! VRS icin konum gonderimi baslatiliyor...")
                    
                    last_gga_time = 0 # Son GGA gönderme zamanı

                    while not self.stop_ntrip and rclpy.ok():
                        # --- 1. GGA GÖNDERME KISMI (YÖNTEM 2) ---
                        current_time = time.time()
                        # Her 5 saniyede bir gönder
                        if (current_time - last_gga_time) > 5.0:
                            if self.latest_gga: # Eğer GPS'ten veri geldiyse
                                try:
                                    s.sendall(self.latest_gga.encode())
                                    last_gga_time = current_time
                                    # self.parse_gga(self.latest_gga)
                                    self.get_logger().debug(f"Guncel GGA sunucuya iletildi {self.latest_gga.strip()}.")
                                except Exception as e:
                                    self.get_logger().warn(f"GGA Gonderilemedi: {e}")
                                    break
                            else:
                                self.get_logger().warn("GPS Fix yok, GGA gonderilemiyor (Bekleniyor...)")

                        # --- 2. RTK VERİSİ ALMA KISMI ---
                        try:
                            data = s.recv(2048)
                            if not data: 
                                self.get_logger().warn("Sunucudan bos veri, baglanti koptu.")
                                break
                            self.ser.write(data)
                        except socket.timeout:
                            # Veri gelmezse timeouta düşer, sorun yok, döngü başa döner ve GGA gönderir
                            continue
                        except Exception as e:
                            self.get_logger().error(f"Soket Okuma Hatasi: {e}")
                            break
                else:
                    self.get_logger().error(f"Baglanti Reddedildi: {response}")

                s.close()
            except Exception as e:
                self.get_logger().error(f"NTRIP Baglanti Hatasi: {e}")
                self.get_logger().info("5 saniye bekleyip tekrar denenecek...")
                time.sleep(5)

    def read_serial(self):
        if not self.ser.is_open: return
        try:
            if not self.isBinary:
                # --- ORiJiNAL ASCII MODU ---
                while self.ser.in_waiting:
                    raw_line = self.ser.readline()
                    try:
                        line = raw_line.decode('ascii', errors='ignore').strip()
                        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                            self.latest_gga = line + "\r\n" 
                        elif "#HEADINGA" in line:
                            self.parse_headinga(line)
                        elif "#BESTPOSA" in line:
                            self.parse_bestposa(line)
                    except Exception:
                        pass
            else:

                # --- YENI BINARY MODU ---
                data = self.ser.read(self.ser.in_waiting or 1)
                # self.get_logger().info(f"Data: {data}")
                if data:
                    self.buffer.extend(data)
                
                # self.get_logger().info(f"Buffer len: {len(self.buffer)}") # DEBUG

                while True:
                    # Minimum paket boyutu kontrolü (Header 28 byte)
                    if len(self.buffer) < 28:
                        break
                    
                    # 1. Sync Bytes Kontrolü: 0xAA 0x44 0x12
                    sync_idx = self.buffer.find(b'\xaa\x44\x12')
                    
                    # 2. ASCII Başlangıç Kontrolü: $
                    ascii_idx = self.buffer.find(b'$')

                    # Eğer ikisi de yoksa, buffer'ı temizle ama son kısmı tut (belki header yarım gelmiştir)
                    if sync_idx == -1 and ascii_idx == -1:
                        # self.get_logger().info("No sync or ascii found, clearing buffer") # DEBUG
                        self.buffer = self.buffer[-3:] 
                        break

                    # Hangisi daha önce geliyorsa onu işle
                    # Eğer sadece biri varsa, diğerinin indexi -1 olduğu için min() fonksiyonunu doğrudan kullanamayız
                    if sync_idx != -1 and (ascii_idx == -1 or sync_idx < ascii_idx):
                        # --- BINARY PAKET ---
                        # Sync öncesindeki çöp veriyi at
                        if sync_idx > 0:
                            self.get_logger().info(f"Dropping {sync_idx} bytes before sync")
                            del self.buffer[:sync_idx]
                        
                        # Header'ı çekmeye çalış
                        if len(self.buffer) < 28:
                            break # Yeterli veri yok, bekle

                        # Header Yapısı (Little Endian <):
                        # Sync(3), HdrLen(1), MsgID(2), MsgType(1), PortAddr(1), MsgLen(2), Seq(2), Idle(1), TimeStatus(1), Week(2), MS(4), RxStat(4), Reserved(2), Vers(2)
                        # MsgID: offset 4, MsgLen: offset 8
                        try:
                            # HdrLen: offset 3 (1 byte)
                            header_len = self.buffer[3]
                            msg_id = struct.unpack_from('<H', self.buffer, 4)[0]
                            msg_len = struct.unpack_from('<H', self.buffer, 8)[0] # Payload length
                            
                            total_len = header_len + msg_len + 4 # +4 CRC
                            
                            # self.get_logger().info(f"Binary Header Found: MsgID={msg_id}, Len={msg_len}, Total={total_len}") 

                            if len(self.buffer) < total_len:
                                # self.get_logger().info(f"Waiting for full packet. Have {len(self.buffer)}/{total_len}")
                                break # Tüm paket gelmedi, bekle

                            # Paketi al
                            packet = self.buffer[:total_len]
                            payload = packet[header_len : header_len + msg_len]
                            
                            # İşle
                            if msg_id == 42: # BESTPOS
                                # self.get_logger().info(f"Parsing BESTPOS Binary (ID 42)")
                                self.parse_binary_bestpos(payload)
                            elif msg_id == 971: # HEADING
                                # self.get_logger().info(f"Parsing HEADING Binary (ID 971)")
                                self.parse_binary_heading(payload)
                            elif msg_id == 1429: # BESTGNSSPOS (Alternatif)
                                #  self.get_logger().info(f"Parsing BESTGNSSPOS Binary (ID 1429)")
                                 self.parse_binary_bestpos(payload) # Yapısı BESTPOS ile aynı
                            else:
                                 self.get_logger().info(f"Unknown MsgID: {msg_id}")
                                 pass
                            
                            # Paketi buffer'dan sil
                            del self.buffer[:total_len]
                            
                        except Exception as e:
                            self.get_logger().error(f"Binary parse hatasi: {e}")
                            del self.buffer[:1] # Bir byte kaydır ve tekrar dene

                    else:
                        # --- ASCII SATIR (GGA) ---
                        # $ işaretinden öncesini at
                        if ascii_idx > 0:
                            del self.buffer[:ascii_idx]
                        
                        # Satır sonu (\n) ara
                        lf_idx = self.buffer.find(b'\n')
                        if lf_idx == -1:
                            break # Satır sonu gelmedi, bekle
                        
                        # Satırı al
                        line_bytes = self.buffer[:lf_idx+1]
                        del self.buffer[:lf_idx+1]
                        
                        try:
                            line = line_bytes.decode('ascii', errors='ignore').strip()
                            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                                self.latest_gga = line + "\r\n"
                        except Exception:
                            pass

        except Exception as e:
            self.get_logger().error(f"Read Serial Error: {e}")

    def parse_gga(self, line):
        try:
            msg = pynmea2.parse(line)
            ros_msg = NavSatFix()
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = self.frame_id
            ros_msg.latitude = msg.latitude
            ros_msg.longitude = msg.longitude
            ros_msg.altitude = msg.altitude

            qual = msg.gps_qual
            # RTD100/M20 Kalite Kodları: 4=Fixed, 5=Float, 1=Single
            if qual == 4:
                ros_msg.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK Fixed
                cov = 0.0001
            elif qual == 5:
                ros_msg.status.status = NavSatStatus.STATUS_SBAS_FIX  # RTK Float
                cov = 0.05
            elif qual == 1:
                ros_msg.status.status = NavSatStatus.STATUS_FIX  # Single/3D Fix
                cov = 2.5
            elif qual == 0 or qual == 6:
                ros_msg.status.status = NavSatStatus.STATUS_NO_FIX  # No Fix
                return  # Don't publish invalid fix
            else:
                ros_msg.status.status = NavSatStatus.STATUS_NO_FIX  # Unknown = No Fix
                cov = 10000.0

            ros_msg.position_covariance = [cov, 0.0, 0.0, 0.0, cov, 0.0, 0.0, 0.0, cov*2]
            ros_msg.position_covariance_type = 2
            self.fix_pub.publish(ros_msg)
        except Exception:
            self.get_logger().error(f"GGA Parse Hatasi: {line}")
            pass

    def _safe_float(self, s: str, default=None):
        try:
            s = s.strip().strip('"')
            return float(s)
        except Exception:
            return default

    def parse_binary_bestpos(self, payload):
        try:
            # BESTPOS Binary: ID 42, Length 72
            # Structure (Little Endian <):
            # SolStat(I), PosType(I), Lat(d), Lon(d), Hgt(d), Und(f), 
            # Datum(I), LatStd(f), LonStd(f), HgtStd(f), StnID(4s), 
            # DiffAge(f), SolAge(f), NumSVs(B), NumSolSVs(B), NumL1(B), NumMulti(B), Reserved(B), ExtSolStat(B), GalBeiMask(B), GPSGloMask(B)
            # Format: < II ddd f I fff 4s ff B B B B B B B B
            # 4+4+8+8+8+4+4+4+4+4+4+4+4+1+1+1+1+1+1+1+1 = 72 bytes
            
            if len(payload) < 72: 
                self.get_logger().error("BESTPOS Binary: Payload too short")
                return

            data = struct.unpack('<IIdddfIfff4sffBBBBBBBB', payload[:72])
            
            sol_stat = data[0]
            pos_type = data[1]
            lat = data[2]
            lon = data[3]
            hgt = data[4]
            # und = data[5]
            # datum = data[6]
            lat_std = data[7]
            lon_std = data[8]
            hgt_std = data[9]
            
            # SOL_COMPUTED = 0
            if sol_stat != 0: 
                self.get_logger().error("BESTPOS Binary: SOL_STAT != 0")
                return

            ros_msg = NavSatFix()
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = self.frame_id
            ros_msg.latitude = lat
            ros_msg.longitude = lon
            ros_msg.altitude = hgt

            # Position Type Mapping (Table 4-2)
            # 16=SINGLE, 17=PSRDIFF, 18=WAAS
            # 32=L1_FLOAT, 34=NARROW_FLOAT, 55=INS_RTKFLOAT
            # 48=L1_INT, 49=WIDE_INT, 50=NARROW_INT, 56=INS_RTKFIXED
            if pos_type in [48, 49, 50, 51, 56, 69]: # Fixed
                 ros_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif pos_type in [4, 5, 6, 32, 33, 34, 55, 68]: # Float
                 ros_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif pos_type in [16, 17, 18, 19, 52, 53, 54]: # Single
                 ros_msg.status.status = NavSatStatus.STATUS_FIX
            elif pos_type == 0:
                 ros_msg.status.status = NavSatStatus.STATUS_NO_FIX
            else:
                 ros_msg.status.status = NavSatStatus.STATUS_FIX

            ros_msg.status.service = NavSatStatus.SERVICE_GPS
            
            cov_lat = float(lat_std) ** 2
            cov_lon = float(lon_std) ** 2
            cov_alt = float(hgt_std) ** 2

            ros_msg.position_covariance = [
                cov_lat, 0.0,    0.0,
                0.0,    cov_lon, 0.0,
                0.0,    0.0,    cov_alt
            ]
            ros_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            # self.get_logger().info(f"GPS Fix: {ros_msg}")
            self.fix_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f"BESTPOS Binary Parse Error: {e}")

    def parse_binary_heading(self, payload):
        try:
            # HEADING Binary: ID 971, Length 44
            # SolStat(I), PosType(I), Len(f), Heading(f), Pitch(f), Rsv(f), 
            # HeadStd(f), PitchStd(f), StnID(4s), 
            # NumSVs(B), NumSolSVs(B), NumObs(B), NumMulti(B), SolSrc(B), ExtSolStat(B), GalBeiMask(B), GPSGloMask(B)
            # Format: < II fff f ff 4s B B B B B B B B
            # 4+4+4+4+4+4+4+4+4+1+1+1+1+1+1+1+1 = 44 bytes
            
            if len(payload) < 44: 
                self.get_logger().error("HEADING Binary: Payload too short")
                return

            data = struct.unpack('<IIffffff4sBBBBBBBB', payload[:44])
            
            sol_stat = data[0]
            pos_type = data[1]
            baseline_len = data[2]
            heading_deg = data[3]
            heading_std_deg = data[6]
            
            # self.get_logger().info(f"Raw Payload: {payload[:16].hex()}...") 
            # self.get_logger().info(f"BinHeading: Stat={sol_stat}, PosType={pos_type}, Len={baseline_len:.3f}, Hdg={heading_deg}, Std={heading_std_deg}") 

            if sol_stat != 0: 
                self.get_logger().error(f"HEADING Binary: SOL_STAT {sol_stat} != 0")
                return 

            # PosType 0 = NONE means no solution
            if pos_type == 0:
                self.get_logger().warn("HEADING Binary: PosType 0 (NONE)")
                return

            yaw_deg = 90.0 - heading_deg 
            if yaw_deg < -180.0: yaw_deg += 360.0
            if yaw_deg > 180.0:  yaw_deg -= 360.0
            
            yaw_rad = math.radians(yaw_deg)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

            std_rad = max(math.radians(heading_std_deg), math.radians(0.2))
            cov = std_rad ** 2

            imu_msg.orientation_covariance = [
                0.01, 0.0,     0.0,
                0.0,     0.01, 0.0,
                0.0,     0.0,     cov
            ]
            
            # Gyro/Accel Unknown
            imu_msg.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            imu_msg.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"HEADING Binary Parse Error: {e}")

    def parse_bestposa(self, line: str):
        try:
            # self.get_logger().info(f"BESTPOSA alindi: {line}")
            # '#BESTPOSA,...;payload*checksum' formatı
            if ';' not in line:
                return

            payload = line.split(';', 1)[1]

            # checksum varsa kopar: '...*ABCDEF'
            if '*' in payload:
                payload = payload.split('*', 1)[0]

            data = payload.split(',')
            if len(data) < 10:
                self.get_logger().warn(f"BESTPOSA payload too short: {payload}")
                return

            sol_status = data[0].strip()      # SOL_COMPUTED / INSUFFICIENT_OBS
            pos_type   = data[1].strip()      # RTK_FIXED / RTK_FLOAT / NONE

            # ✅ önce sol_status kontrolü (patlamayı önler)
            if sol_status != "SOL_COMPUTED":
                # Debug istersen aç:
                # self.get_logger().debug(f"BESTPOSA ignored: {sol_status},{pos_type}")
                return

            # Bu formatta: [2]=lat, [3]=lon, [4]=hgt
            latitude  = self._safe_float(data[2])
            longitude = self._safe_float(data[3])
            height    = self._safe_float(data[4])
            if latitude is None or latitude == 0.0 or longitude is None or longitude == 0.0 or height is None or height == 0.0:
                self.get_logger().warn(f"BESTPOSA konum verisi hatali: {payload}")
                return
            # std alanları bu tip BESTPOSA'da genelde şuralarda:
            # ... WGS84, undulation, datum_id? ... sonra lat_std, lon_std, hgt_std
            # Senin örneğinde:
            # ..., WGS84,0.0000,0.0000,0.0000,"",0.000,0.003,0,0,...
            # Burada 0.000 ve 0.003 gibi std'ler var ama indeks cihazına göre değişebilir.
            #
            # En güvenlisi: payload içinden std adaylarını "sonlarda küçük değerler" olarak seçmek.
            # Ama basit olsun diye: önce bilinen indeksleri dene, yoksa fallback koy.
            lat_std = None
            lon_std = None
            hgt_std = None

            # Deneme-1: NovAtel benzeri layout (bazı cihazlarda [10],[11],[12])
            if len(data) > 9:
                lat_std = self._safe_float(data[7])
                lon_std = self._safe_float(data[8])
                hgt_std = self._safe_float(data[9])

            # Deneme-2: Senin örneğe benzer layout: std'ler "" dan sonra geliyor olabilir
            # "" alanını bulup sonraki 3 sayıyı almayı dene
            if lat_std is None or lon_std is None:
                try:
                    empty_idx = data.index('""') if '""' in data else data.index('""'.strip())
                except ValueError:
                    empty_idx = None
                if empty_idx is not None and len(data) > empty_idx + 3:
                    lat_std2 = self._safe_float(data[empty_idx + 1])
                    lon_std2 = self._safe_float(data[empty_idx + 2])
                    hgt_std2 = self._safe_float(data[empty_idx + 3])
                    # sadece mantıklıysa al (None değilse)
                    if lat_std2 is not None and lon_std2 is not None:
                        lat_std, lon_std, hgt_std = lat_std2, lon_std2, (hgt_std2 if hgt_std2 is not None else 5.0)

            # Fallback: std bulamazsak makul bir şey koy
            if lat_std is None or lon_std is None:
                # RTK_FIXED ise çok küçük, değilse daha büyük
                if pos_type == "RTK_FIXED":
                    lat_std, lon_std, hgt_std = 0.02, 0.02, 0.05
                elif pos_type == "RTK_FLOAT":
                    lat_std, lon_std, hgt_std = 0.20, 0.20, 0.50
                else:
                    lat_std, lon_std, hgt_std = 2.0, 2.0, 5.0

            if hgt_std is None:
                hgt_std = 5.0

            # Avoid zero/tiny std devs that make the filter overconfident.
            if "FIX" in pos_type or "INT" in pos_type:
                min_std = 0.02
            elif "FLOAT" in pos_type:
                min_std = 0.20
            else:
                min_std = 1.00
            lat_std = max(abs(float(lat_std)), min_std)
            lon_std = max(abs(float(lon_std)), min_std)
            hgt_std = max(abs(float(hgt_std)), min_std * 2.5)

            # NavSatFix oluştur
            ros_msg = NavSatFix()
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = self.frame_id
            ros_msg.latitude = float(latitude)
            ros_msg.longitude = float(longitude)
            ros_msg.altitude = float(height)
            
            # RTK Fixed Modes
            if pos_type in ["RTK_FIXED", "L1_INT", "WIDE_INT", "NARROW_INT", "RTK_DIRECT_INS", "INS_RTKFIXED", "PPP"]:
                ros_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            # RTK Float Modes
            elif pos_type in ["RTK_FLOAT", "FLOATCONV", "WIDELANE", "NARROWLANE", "L1_FLOAT", "IONOFREE_FLOAT", "NARROW_FLOAT", "INS_RTKFLOAT", "PPP_CONVERGING"]:
                ros_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            # Single / DGPS Modes
            elif pos_type in ["SINGLE", "PSRDIFF", "WAAS", "INS_SBAS", "INS_PSRSP", "INS_PSRDIFF", "PROPAGATED"]:
                ros_msg.status.status = NavSatStatus.STATUS_FIX
            elif pos_type == "NONE":
                 ros_msg.status.status = NavSatStatus.STATUS_NO_FIX
            else:
                # Fallback
                ros_msg.status.status = NavSatStatus.STATUS_FIX
            ros_msg.status.service = NavSatStatus.SERVICE_GPS
            
            cov_lat = float(lat_std) ** 2
            cov_lon = float(lon_std) ** 2
            cov_alt = float(hgt_std) ** 2

            ros_msg.position_covariance = [
                cov_lat, 0.0,    0.0,
                0.0,    cov_lon, 0.0,
                0.0,    0.0,    cov_alt
            ]
            ros_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(ros_msg)

        except Exception as e:
            self.get_logger().error(f"BESTPOSA parse hatası: {e} line={line[:120]}")

    def parse_headinga(self, line):
        try:
            parts = line.split(';')
            if len(parts) != 2: return
            data = parts[1].split(',')
            
            sol_stat = data[0]
            heading_deg = float(data[3])
            heading_std_deg = float(data[6])
            # self.get_logger().info(f"Heading: {heading_deg:.2f}°,Sol Stat: {sol_stat},cov: {heading_std_deg:.10f}°")
            if sol_stat != "SOL_COMPUTED" or heading_std_deg == 0.0: return

            yaw_deg = 90.0 - heading_deg 
            if yaw_deg < -180.0: yaw_deg += 360.0
            if yaw_deg > 180.0:  yaw_deg -= 360.0
            
            yaw_rad = math.radians(yaw_deg)

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)
            # self.get_logger().info(f"Heading: {heading_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

            std_rad = max(math.radians(heading_std_deg), math.radians(0.2))  # min 0.2°
            cov = std_rad ** 2
            # self.get_logger().info(f"Heading Std: {heading_std_deg:.2f}°, Cov: {cov:.6f} rad²")
            imu_msg.orientation_covariance = [
                                                0.01, 0.0,     0.0,
                                                0.0,     0.01, 0.0,
                                                0.0,     0.0,     cov
                                            ]

            # Gyro YOK: spec'e göre [0] = -1 yap
            imu_msg.angular_velocity_covariance = [
                                                    -1.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0
                                                ]

            # Accel YOK: spec'e göre [0] = -1 yap
            imu_msg.linear_acceleration_covariance = [
                                                        -1.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0,
                                                        0.0, 0.0, 0.0
                                                    ]

            
            self.imu_pub.publish(imu_msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = rtd100_driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_ntrip = True
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
