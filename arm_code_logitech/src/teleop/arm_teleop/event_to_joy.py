import array
import os
import select
import struct
from typing import Dict

import fcntl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


EV_KEY = 0x01
EV_ABS = 0x03

ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05
ABS_THROTTLE = 0x06
ABS_RUDDER = 0x07
ABS_WHEEL = 0x08
ABS_GAS = 0x09
ABS_BRAKE = 0x0A
ABS_HAT0X = 0x10
ABS_HAT0Y = 0x11

BTN_SOUTH = 0x130
BTN_EAST = 0x131
BTN_NORTH = 0x133
BTN_WEST = 0x134
BTN_TL = 0x136
BTN_TR = 0x137
BTN_TL2 = 0x138
BTN_TR2 = 0x139
BTN_SELECT = 0x13A
BTN_START = 0x13B
BTN_MODE = 0x13C
BTN_THUMBL = 0x13D
BTN_THUMBR = 0x13E

EVENT_STRUCT = struct.Struct("llHHI")
ABS_INFO_STRUCT = struct.Struct("iiiiii")
EVIOCGABS_BASE = 0x80184540


def _eviocgabs(axis_code: int) -> int:
    return EVIOCGABS_BASE + axis_code


class EventToJoy(Node):
    def __init__(self) -> None:
        super().__init__("event_to_joy")

        self.declare_parameter("event_device", "/dev/input/event0")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("axis_smoothing_alpha", 0.35)
        self.declare_parameter("debug_raw_events", False)

        configured_event_device = str(self.get_parameter("event_device").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.axis_smoothing_alpha = float(self.get_parameter("axis_smoothing_alpha").value)
        self.debug_raw_events = bool(self.get_parameter("debug_raw_events").value)

        self.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.smoothed_axes = list(self.axes)
        self.buttons = [0] * 11
        self._logged_unknown_codes = set()

        # F310 DirectInput on this Linux/WSL path reports:
        #   ABS_X, ABS_Y, ABS_Z, ABS_RZ, ABS_HAT0X, ABS_HAT0Y
        # Treat Z/RZ as the right-stick pair so the original Logitech mapping works.
        self.abs_to_axis_index: Dict[int, int] = {
            ABS_X: 0,
            ABS_Y: 1,
            ABS_Z: 3,
            ABS_RZ: 4,
            ABS_BRAKE: 2,
            ABS_GAS: 5,
            ABS_THROTTLE: 2,
            ABS_RUDDER: 5,
            ABS_HAT0X: 6,
            ABS_HAT0Y: 7,
        }
        self.key_to_button_index: Dict[int, int] = {
            BTN_SOUTH: 0,
            BTN_EAST: 1,
            BTN_WEST: 2,
            BTN_NORTH: 3,
            BTN_TL: 4,
            BTN_TR: 5,
            BTN_SELECT: 6,
            BTN_START: 7,
            BTN_MODE: 8,
            BTN_THUMBL: 9,
            BTN_THUMBR: 10,
        }
        self.trigger_button_to_axis: Dict[int, int] = {
            BTN_TL2: 2,
            BTN_TR2: 5,
            294: 2,
            295: 5,
        }
        self.axis_sign: Dict[int, float] = {
            ABS_X: 1.0,
            ABS_Y: -1.0,
            ABS_Z: 1.0,
            ABS_RZ: 1.0,
            ABS_BRAKE: 1.0,
            ABS_GAS: 1.0,
            ABS_THROTTLE: 1.0,
            ABS_RUDDER: 1.0,
            ABS_HAT0X: 1.0,
            ABS_HAT0Y: -1.0,
        }

        self.event_device = self._resolve_event_device(configured_event_device)
        self.abs_ranges = self._load_abs_ranges()
        self.fd = os.open(self.event_device, os.O_RDONLY | os.O_NONBLOCK)

        self.publisher = self.create_publisher(Joy, joy_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._poll_and_publish)

        self.get_logger().info(
            f"Reading Linux event device {self.event_device} and publishing /{joy_topic}."
        )

    def _resolve_event_device(self, configured_event_device: str) -> str:
        if os.path.exists(configured_event_device):
            return configured_event_device

        input_dir = "/dev/input"
        if os.path.isdir(input_dir):
            for entry in sorted(os.listdir(input_dir)):
                if entry.startswith("event"):
                    candidate = os.path.join(input_dir, entry)
                    if os.path.exists(candidate):
                        self.get_logger().warn(
                            f"Configured event device '{configured_event_device}' not found. Falling back to '{candidate}'."
                        )
                        return candidate

        raise RuntimeError(f"No Linux event device found at '{configured_event_device}' or under /dev/input/event*.")

    def _load_abs_ranges(self) -> Dict[int, tuple[int, int]]:
        ranges: Dict[int, tuple[int, int]] = {}

        try:
            fd = os.open(self.event_device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            raise RuntimeError(f"Failed to open event device '{self.event_device}': {exc}") from exc

        try:
            for axis_code in self.abs_to_axis_index:
                buffer = array.array("b", b"\x00" * ABS_INFO_STRUCT.size)
                try:
                    fcntl.ioctl(fd, _eviocgabs(axis_code), buffer, True)
                except OSError:
                    continue
                value, minimum, maximum, fuzz, flat, resolution = ABS_INFO_STRUCT.unpack(buffer.tobytes())
                del value, fuzz, flat, resolution
                ranges[axis_code] = (minimum, maximum)
        finally:
            os.close(fd)

        self.get_logger().info(f"Event axis mapping: {self.abs_to_axis_index}")
        return ranges

    def _normalize_axis(self, axis_code: int, raw_value: int) -> float:
        if axis_code in (ABS_HAT0X, ABS_HAT0Y):
            return float(raw_value) * self.axis_sign.get(axis_code, 1.0)

        if axis_code in (ABS_BRAKE, ABS_GAS, ABS_THROTTLE, ABS_RUDDER):
            minimum, maximum = self.abs_ranges.get(axis_code, (0, 255))
            span = maximum - minimum
            if span <= 0:
                return 1.0
            ratio = (float(raw_value) - minimum) / span
            ratio = max(0.0, min(1.0, ratio))
            return 1.0 - (2.0 * ratio)

        minimum, maximum = self.abs_ranges.get(axis_code, (-32768, 32767))
        span = maximum - minimum
        if span <= 0:
            return 0.0

        center = minimum + (span / 2.0)
        normalized = (float(raw_value) - center) / (span / 2.0)
        normalized = max(-1.0, min(1.0, normalized))
        normalized *= self.axis_sign.get(axis_code, 1.0)
        if abs(normalized) < self.deadzone:
            return 0.0
        return normalized

    def _poll_and_publish(self) -> None:
        while True:
            readable, _, _ = select.select([self.fd], [], [], 0.0)
            if not readable:
                break

            try:
                data = os.read(self.fd, EVENT_STRUCT.size * 64)
            except BlockingIOError:
                break

            if not data:
                break

            for offset in range(0, len(data), EVENT_STRUCT.size):
                chunk = data[offset : offset + EVENT_STRUCT.size]
                if len(chunk) != EVENT_STRUCT.size:
                    continue
                _, _, event_type, code, value = EVENT_STRUCT.unpack(chunk)

                if event_type == EV_ABS and code in self.abs_to_axis_index:
                    axis_index = self.abs_to_axis_index[code]
                    self.axes[axis_index] = self._normalize_axis(code, int(value))
                elif event_type == EV_KEY and code in self.key_to_button_index:
                    button_index = self.key_to_button_index[code]
                    self.buttons[button_index] = 1 if value else 0
                elif event_type == EV_KEY and code in self.trigger_button_to_axis:
                    axis_index = self.trigger_button_to_axis[code]
                    # Emulate joy_node trigger axes: released=1.0, pressed=-1.0
                    self.axes[axis_index] = -1.0 if value else 1.0
                elif self.debug_raw_events and event_type in (EV_KEY, EV_ABS):
                    key = (event_type, code)
                    if key not in self._logged_unknown_codes or value:
                        self.get_logger().info(
                            f"Unmapped raw event: type={event_type} code={code} value={value}"
                        )
                        self._logged_unknown_codes.add(key)

        for axis_index, raw_value in enumerate(self.axes):
            # D-pad axes are already discrete and should stay snappy.
            if axis_index in (6, 7):
                self.smoothed_axes[axis_index] = raw_value
                continue
            self.smoothed_axes[axis_index] += (
                raw_value - self.smoothed_axes[axis_index]
            ) * self.axis_smoothing_alpha

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self.smoothed_axes)
        msg.buttons = list(self.buttons)
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        try:
            os.close(self.fd)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EventToJoy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
