import argparse
import time
import sys
import os
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from natnet import DataDescriptions, DataFrame, NatNetClient, RigidBody


id_to_name = {}
has_frame = False
# 帧率统计开关
rate_enabled = False
# 串口与目标刚体配置
selected_rb_name = None
serial_conn = None
# ExpressLRS CRC table
crc8tab = [0] * 256


def crc8_init(poly: int):
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) & 0xFF) ^ (poly if (crc & 0x80) else 0)
        crc8tab[i] = crc & 0xFF


def crc8_calc(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc = crc8tab[(crc ^ b) & 0xFF]
    return crc


def generate_crsf_packet_rc(RCH: list) -> bytes:
    """Port of generate_crsf_packet_rc from expresslrs.c
    RCH: list of 16 uint16 values (only lower 11 bits used)
    returns: bytes of length 26 (CRSF packet)
    """
    total_len = 26
    payload_len = 22
    buf = bytearray(total_len)
    buf[0] = 0xEE
    buf[1] = total_len - 2
    buf[2] = 0x16

    bit_buffer = 0
    bits_in_buffer = 0
    byte_index = 0
    for ch in range(16):
        # Use 10-bit source values left-shifted by 1 to fill 11-bit channel field,
        # matching generate_crsf_packet_rc_10bit behavior in expresslrs.c
        value = (int(RCH[ch]) << 1) & 0x07FF
        bit_buffer |= (value << bits_in_buffer)
        bits_in_buffer += 11
        while bits_in_buffer >= 8 and byte_index < payload_len:
            buf[3 + byte_index] = bit_buffer & 0xFF
            bit_buffer >>= 8
            bits_in_buffer -= 8
            byte_index += 1

    while byte_index < payload_len:
        buf[3 + byte_index] = bit_buffer & 0xFF
        bit_buffer >>= 8
        byte_index += 1

    # CRC over buf[2]..buf[24] (1 + payload_len bytes)
    crc = crc8_calc(bytes(buf[2 : 2 + 1 + payload_len]), 0)
    buf[3 + payload_len] = crc
    return bytes(buf)

# 频率统计相关全局变量（运行时初始化）
freq_window = 1.0  # 秒
freq_source = "client"  # "client" 或 "server"
last_report_time = None  # 使用 monotonic 进行客户端到达频率统计
last_report_frames = 0
last_server_ts_for_report = None
last_server_frames = 0
last_frame_number = None
lost_frames_total = 0
lost_frames_report_mark = 0


def receive_desc(desc: DataDescriptions):
    # Map top-level rigid body IDs to names (if available)
    for rb in desc.rigid_bodies or []:
        if rb is not None:
            id_to_name[rb.id_num] = rb.name or f"RigidBody#{rb.id_num}"


def _fmt_vec(v):
    return f"({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f})"


def _fmt_quat(q):
    return f"({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})"


def _send_rb_over_serial(rb: RigidBody, name):
    """格式化并通过串口发送刚体数据（CSV 行）。"""
    global serial_conn
    try:
        pos = rb.pos
        rot = rb.rot  # 四元数 [qx, qy, qz, qw]
        qw, qx, qy, qz = rot[3], rot[0], rot[1], rot[2]
        yaw = math.atan2(2 * (qw * qz - qx * qy), 1 - 2 * (qx**2 + qz**2))
        yaw_deg = math.degrees(yaw) % 360
        yaw_bits = int((yaw_deg / 360.0) * 4096) & 0xFFF  # 12位

        if abs(pos[0]) > 8.0 or abs(pos[1]) > 8.0 or abs(pos[2]) > 8.0:
            # 超出范围则不发送
            return
        pos_scale = 65536.0
        px = int((pos[0] + 8.0) * pos_scale) & 0xFFFFF  # 20位
        py = int((pos[1] + 8.0) * pos_scale) & 0xFFFFF
        pz = int((pos[2] + 8.0) * pos_scale) & 0xFFFFF
        id_bits = rb.id_num & 0xFF  # 8位

        # 打包80位: px(20) + py(20) + pz(20) + yaw(12) + id(8)
        bit_buffer = (px << 60) | (py << 40) | (pz << 20) | (yaw_bits << 8) | id_bits

        # 分成8个10位通道 (RCH 0-3,5-8)
        RCH = [0] * 16
        for i in range(8):
            shift = 70 - i * 10
            RCH[i if i < 4 else i + 1] = (bit_buffer >> shift) & 0x3FF
        RCH[4] = 900

        # 初始化 CRC 表（若尚未初始化）并生成 CRSF 包
        try:
            if crc8tab[1] == 0:
                crc8_init(0xD5)
        except Exception:
            crc8_init(0xD5)

        pkt = generate_crsf_packet_rc(RCH)

        if serial_conn is not None:
            try:
                serial_conn.write(pkt)
                print('pos:',pos,'yaw:',yaw_deg)
            except Exception:
                print("Warning: failed to write CRSF packet to serial port")
        else:
            # 未配置串口时打印十六进制供调试
            print("[CRSF-Sim]", pkt.hex())
    except Exception:
        pass


def receive_frame(df: DataFrame):
    global has_frame
    global last_report_time, last_report_frames
    global last_server_ts_for_report, last_server_frames
    global last_frame_number, lost_frames_total, lost_frames_report_mark
    global freq_window, freq_source
    has_frame = True
    arrival_now_mono = time.perf_counter()
    # 帧计数（用于频率统计，仅在启用时）
    if rate_enabled:
        last_report_frames += 1

    # 根据帧号估算丢帧（客户端侧检测）
    try:
        fn = df.prefix.frame_number
        if last_frame_number is not None:
            delta = int(fn) - int(last_frame_number)
            if delta > 1:
                lost_frames_total += (delta - 1)
        last_frame_number = fn
    except Exception:
        pass
    # Print top-level rigid bodies
    for rb in df.rigid_bodies or []:
        name = id_to_name.get(rb.id_num, f"RigidBody#{rb.id_num}")
        pos = _fmt_vec(rb.pos)
        rot = _fmt_quat(rb.rot)
        valid = rb.tracking_valid if rb.tracking_valid is not None else True
        err = rb.marker_error if rb.marker_error is not None else float("nan")
        # 若设置了 selected_rb_name，则发送匹配的刚体数据并跳出
        global selected_rb_name
        if selected_rb_name and name == selected_rb_name:
            _send_rb_over_serial(rb, name)
            # 只发送一个刚体，找到后结束搜索
            return


    # 频率统计：按窗口打印 FPS（客户端到达或服务器时间戳），仅在启用时
    if rate_enabled:
        if freq_source == "server" and getattr(df, "suffix", None) is not None:
            ts = df.suffix.timestamp
            if ts is not None:
                if last_server_ts_for_report is None:
                    last_server_ts_for_report = ts
                    last_server_frames = last_report_frames
                elif (ts - last_server_ts_for_report) >= float(freq_window):
                    df_count = last_report_frames - last_server_frames
                    dt = ts - last_server_ts_for_report
                    if dt > 0:
                        fps = df_count / dt
                        print(f"[Rate] Server timestamp FPS: {fps:.2f}")
                    last_server_ts_for_report = ts
                    last_server_frames = last_report_frames
        else:
            now = arrival_now_mono
            if last_report_time is None:
                last_report_time = now
            elif (now - last_report_time) >= float(freq_window):
                df_count = last_report_frames
                dt = now - last_report_time
                if dt > 0:
                    fps = df_count / dt
                    # 同时打印到达频率与窗口内估算的丢帧数
                    lost_since_last = lost_frames_total - lost_frames_report_mark
                    if lost_since_last > 0:
                        print(f"[Rate] Client arrival FPS: {fps:.2f} (lost ~{lost_since_last})")
                    else:
                        print(f"[Rate] Client arrival FPS: {fps:.2f}")
                last_report_time = now
                last_report_frames = 0
                lost_frames_report_mark = lost_frames_total


def _try_connect_and_probe(server_ip: str, local_ip: str, use_multicast: bool, probe_seconds: float = 1.5):
    """
    尝试以给定模式连接并在短时间窗口内探测是否收到数据帧；成功则返回已连接的 client，失败则返回 None。
    """
    global has_frame
    has_frame = False

    client = NatNetClient(
        server_ip_address=server_ip,
        local_ip_address=local_ip,
        use_multicast=use_multicast,
    )
    client.on_data_description_received_event.handlers.append(receive_desc)
    client.on_data_frame_received_event.handlers.append(receive_frame)

    try:
        client.connect(timeout=1.0)
        client.request_modeldef()
        end_time = time.time() + float(probe_seconds)
        while time.time() < end_time and not has_frame:
            time.sleep(0.02)
            client.update_sync()
        if has_frame:
            return client
    except Exception:
        pass

    # 失败则清理
    try:
        client.shutdown()
    except Exception:
        pass
    return None


def _parse_duration_arg(val: str):
    """
    将命令行 duration 转换为秒数；返回 None 表示无限。
    接受：正数秒；"inf"/"infinite"/"unlimited"/"none"；"0"/负数视为无限。
    """
    s = str(val).strip().lower()
    if s in ("inf", "infinite", "unlimited", "none"):
        return None
    try:
        f = float(s)
        if f <= 0:
            return None
        return f
    except Exception:
        # 无法解析则回退到默认 10 秒
        return 10.0


def main():
    parser = argparse.ArgumentParser(
        description="NatNet: 获取并打印刚体位姿数据"
    )
    parser.add_argument(
        "--server-ip",
        default="127.0.0.1",
        help="Motive/NatNet 服务器 IP 地址 (默认: 127.0.0.1)",
    )
    parser.add_argument(
        "--local-ip",
        default="127.0.0.1",
        help="本地网络接口 IP 地址，用于接收数据 (默认: 127.0.0.1)",
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "unicast", "multicast"],
        default="auto",
        help="连接模式：auto(自动探测)/unicast/" "multicast (默认: auto)",
    )
    parser.add_argument(
        "--duration",
        type=str,
        default="inf",
        help="运行时长(秒)。支持 'inf'/'unlimited'/0 表示无限运行",
    )
    parser.add_argument(
        "--async",
        dest="async_mode",
        action="store_true",
        help="使用异步事件驱动接收（按服务器频率触发），无需轮询",
    )
    parser.add_argument(
        "--rate",
        action="store_true",
        help="启用帧率统计输出（默认关闭）",
    )
    parser.add_argument(
        "--rb-name",
        type=str,
        default=None,
        help="要通过串口发送的刚体名称（必须与 Motive 中名称一致）",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default=None,
        help="串口设备 (例如 COM3)，未指定时仅打印而不发送",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="串口波特率，默认 115200",
    )
    parser.add_argument(
        "--freq-window",
        type=float,
        default=1.0,
        help="统计窗口(秒)，默认 1.0 秒打印一次 FPS",
    )
    parser.add_argument(
        "--freq-source",
        choices=["client", "server"],
        default="client",
        help="FPS 统计来源：client(到达时间) 或 server(数据帧时间戳)",
    )

    args = parser.parse_args()

    # 初始化频率统计参数（仅在启用时生效）
    global rate_enabled, freq_window, freq_source, last_report_time, last_report_frames
    global last_server_ts_for_report, last_server_frames
    rate_enabled = bool(args.rate)
    freq_window = float(args.freq_window)
    freq_source = args.freq_source
    last_report_time = None
    last_report_frames = 0
    last_server_ts_for_report = None
    last_server_frames = 0

    # 串口与目标刚体设置
    global selected_rb_name, serial_conn
    selected_rb_name = args.rb_name
    serial_conn = None
    if args.serial_port:
        try:
            import serial

            serial_conn = serial.Serial(args.serial_port, args.baud, timeout=0.1)
            print(f"Opened serial port {args.serial_port} @ {args.baud}")
        except Exception as ex:
            serial_conn = None
            print(f"Warning: failed to open serial port {args.serial_port}: {ex}")

    # 解析运行时长
    duration_seconds = _parse_duration_arg(args.duration)
    infinite_run = duration_seconds is None

    # 解析目标模式
    desired_mode = args.mode

    chosen_client = None
    chosen_mode = None

    if desired_mode == "unicast":
        print(
            f"Probing unicast to server {args.server_ip} from local {args.local_ip}"
        )
        chosen_client = _try_connect_and_probe(args.server_ip, args.local_ip, False)
        chosen_mode = "unicast" if chosen_client else None
    elif desired_mode == "multicast":
        print(
            f"Probing multicast to server {args.server_ip} from local {args.local_ip}"
        )
        chosen_client = _try_connect_and_probe(args.server_ip, args.local_ip, True)
        chosen_mode = "multicast" if chosen_client else None
    else:
        # auto: 先尝试单播，失败则尝试组播
        print(
            f"Auto mode: probing unicast then multicast (server {args.server_ip}, local {args.local_ip})"
        )
        chosen_client = _try_connect_and_probe(args.server_ip, args.local_ip, False)
        chosen_mode = "unicast" if chosen_client else None
        if chosen_client is None:
            chosen_client = _try_connect_and_probe(
                args.server_ip, args.local_ip, True
            )
            chosen_mode = "multicast" if chosen_client else None

    if chosen_client is None:
        raise RuntimeError(
            "无法建立数据流连接：单播/组播均未在探测窗口内收到帧。请检查 Motive 的 Streaming 设置与网络配置。"
        )

    print(f"Using mode: {chosen_mode}")

    # 已获得连接的 client，继续按剩余时长运行
    with chosen_client:
        info = chosen_client.server_info
        if info is None:
            # 主动触发一次同步处理（通常无需，但保守做法）
            chosen_client.update_sync()
            info = chosen_client.server_info

        if info is None:
            print("未获取到服务器信息。请确认服务器在线且开启 Streaming。")
            return

        print("=== NatNet Server Info ===")
        print(f"Application: {info.application_name}")
        print(
            f"Server Version: {info.server_version.major}.{info.server_version.minor}.{info.server_version.build}.{info.server_version.revision}"
        )
        print(
            f"NatNet Protocol: {info.nat_net_protocol_version.major}.{info.nat_net_protocol_version.minor}.{info.nat_net_protocol_version.build}.{info.nat_net_protocol_version.revision}"
        )

        print("\n=== Client Connection ===")
        print(f"Server IP: {chosen_client.server_ip_address}")
        print(f"Local  IP: {chosen_client.local_ip_address}")
        print(f"Command Port: {chosen_client.command_port}")
        print(f"Data    Port: {chosen_client.data_port}")
        print(f"Multicast: {chosen_client.use_multicast}")

        # 能否切换协议主版本（仅单播、NatNet >= 4.x）
        print(f"Can change protocol version: {chosen_client.can_change_protocol_version}")
        if chosen_client.can_change_protocol_version:
            print(
                "提示: 支持通过 `client.protocol_version = Version(x, y)` 切换主版本（不在此示例中执行）。"
            )

        chosen_client.request_modeldef()
        
        if infinite_run:
            print("Running with no time limit. Press Ctrl+C to stop.")

        if args.async_mode:
            # 异步模式：后台线程接收数据，按到达频率触发回调
            chosen_client.run_async()
            try:
                if infinite_run:
                    while True:
                        time.sleep(0.1)
                else:
                    end_time = time.time() + float(duration_seconds)
                    while time.time() < end_time:
                        time.sleep(0.1)
            except KeyboardInterrupt:
                pass
            finally:
                chosen_client.stop_async()
        else:
            # 同步模式：轻微睡眠避免忙轮询，按需要调用 update_sync
            try:
                if infinite_run:
                    while True:
                        time.sleep(0.01)
                        chosen_client.update_sync()
                else:
                    end_time = time.time() + float(duration_seconds)
                    while time.time() < end_time:
                        time.sleep(0.01)
                        chosen_client.update_sync()
            except KeyboardInterrupt:
                pass
        # 关闭串口（如有）
        try:
            if serial_conn is not None:
                serial_conn.close()
                print("Serial port closed")
        except Exception:
            pass


if __name__ == "__main__":
    main()
# python D:\MotionCapturePython\python-natnet-client\example\get_rigid_bodies.py  --async --serial-port=COM5 --baud=115200  --rb-name=wrench