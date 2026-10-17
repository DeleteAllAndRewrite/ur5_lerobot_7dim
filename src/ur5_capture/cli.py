import argparse
import json

from .config import load_config


def main():
    parser = argparse.ArgumentParser(description="UR5 + SpaceMouse + three RealSense RGB views")
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("validate", "record"):
        command = sub.add_parser(name)
        command.add_argument("--config", default="configs/ur5_three_rgb.yaml")
        if name == "record":
            command.add_argument("--unattended", action="store_true", help="Mock mode only")
    sub.add_parser("devices")
    args = parser.parse_args()
    if args.command == "devices":
        import pyrealsense2 as rs
        import pyspacemouse
        for device in rs.context().query_devices():
            print("RealSense:", device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
        try:
            devices = pyspacemouse.get_connected_devices()
        except RuntimeError as exc:
            parser.exit(1, f"SpaceMouse discovery failed: {exc}\n"
                        "On Debian/Ubuntu, install the native HID backend: "
                        "sudo apt-get install libhidapi-hidraw0\n")
        print("SpaceMouse:", devices)
        return
    cfg = load_config(args.config)
    if args.command == "validate":
        print(cfg.model_dump_json(indent=2))
        print("Config valid; hardware not opened")
    else:
        from .record import record
        try:
            print(json.dumps(record(cfg, args.unattended), indent=2))
        except KeyboardInterrupt:
            print("Interrupted. Current episode discarded; completed episodes retained.")
            raise SystemExit(130)


if __name__ == "__main__":
    main()
