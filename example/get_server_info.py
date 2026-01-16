import argparse
import os
import sys

# 优先使用当前工作区的 natnet 包
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from natnet import NatNetClient


def main():
    parser = argparse.ArgumentParser(description="NatNet: 获取服务器与协议版本信息")
    parser.add_argument(
        "--server-ip",
        default="127.0.0.1",
        help="Motive/NatNet 服务器 IP 地址 (默认: 127.0.0.1)",
    )
    parser.add_argument(
        "--local-ip",
        default="127.0.0.1",
        help="本地网络接口 IP 地址 (默认: 127.0.0.1)",
    )
    parser.add_argument(
        "--multicast",
        action="store_true",
        help="使用组播模式连接（默认关闭，单播）",
    )

    args = parser.parse_args()

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=bool(args.multicast),
    )

    print(
        f"Connecting to server {args.server_ip} from local {args.local_ip} (multicast={bool(args.multicast)})"
    )
    with client:
        # 连接完成后，`server_info` 会在收到 NAT_SERVERINFO 包后填充
        info = client.server_info
        if info is None:
            # 主动触发一次同步处理（通常无需，但保守做法）
            client.update_sync()
            info = client.server_info

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
        print(f"Server IP: {client.server_ip_address}")
        print(f"Local  IP: {client.local_ip_address}")
        print(f"Command Port: {client.command_port}")
        print(f"Data    Port: {client.data_port}")
        print(f"Multicast: {client.use_multicast}")

        # 能否切换协议主版本（仅单播、NatNet >= 4.x）
        print(f"Can change protocol version: {client.can_change_protocol_version}")
        if client.can_change_protocol_version:
            print(
                "提示: 支持通过 `client.protocol_version = Version(x, y)` 切换主版本（不在此示例中执行）。"
            )


if __name__ == "__main__":
    main()
