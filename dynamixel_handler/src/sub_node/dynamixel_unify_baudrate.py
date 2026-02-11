#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

try:
    import _mylib_dynamixel as dxl
except ImportError:
    dxl = None


BAUDRATE_INDEX_MAP = {
    9600: dxl.BAUDRATE_INDEX_9600 if dxl else 0,
    57600: dxl.BAUDRATE_INDEX_57600 if dxl else 1,
    115200: dxl.BAUDRATE_INDEX_115200 if dxl else 2,
    1000000: dxl.BAUDRATE_INDEX_1M if dxl else 3,
    2000000: dxl.BAUDRATE_INDEX_2M if dxl else 4,
    3000000: dxl.BAUDRATE_INDEX_3M if dxl else 5,
    4000000: dxl.BAUDRATE_INDEX_4M if dxl else 6,
    4500000: dxl.BAUDRATE_INDEX_4M5 if dxl else 7,
    6000000: dxl.BAUDRATE_INDEX_6M if dxl else 8,
    10500000: dxl.BAUDRATE_INDEX_10M5 if dxl else 9,
}

BAUDRATE_LIST = [
    9600,
    57600,
    115200,
    1000000,
    2000000,
    3000000,
    4000000,
    4500000,
    6000000,
    10500000,
]


class DynamixelUnifyBaudrateNode(Node):
    def __init__(self):
        super().__init__("dynamixel_unify_baudrate_node")

        self.declare_parameter("min_id", 0)
        self.declare_parameter("max_id", 50)
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("min_search_baudrate", 57600)
        self.declare_parameter("max_search_baudrate", 4000000)
        self.declare_parameter("target_baudrate", 1000000)
        self.declare_parameter("latency_timer", 16)

        self.id_min = int(self.get_parameter("min_id").value)
        self.id_max = int(self.get_parameter("max_id").value)
        self.device_name = str(self.get_parameter("device_name").value)
        self.baudrate_min = int(self.get_parameter("min_search_baudrate").value)
        self.baudrate_max = int(self.get_parameter("max_search_baudrate").value)
        self.baudrate_target = int(self.get_parameter("target_baudrate").value)
        self.latency_timer = int(self.get_parameter("latency_timer").value)

        self.comm = dxl.DynamixelCommunicator()
        self.comm.set_port_handler(self.device_name)
        self.comm.set_retry_config(2, 5)

    def run(self):
        if self.baudrate_target not in BAUDRATE_INDEX_MAP:
            self.get_logger().error(f"Invalid baudrate {self.baudrate_target}")
            return

        dyn_baudrate = BAUDRATE_INDEX_MAP[self.baudrate_target]
        found_ids = []
        found_id_set = set()

        for baudrate in BAUDRATE_LIST:
            if baudrate < self.baudrate_min or self.baudrate_max < baudrate:
                continue

            self.get_logger().info("                                              ")
            self.get_logger().info(
                f"=== Searching, baudrate:'{baudrate}', id:[{self.id_min}]~[{self.id_max}] ==="
            )

            self.comm.set_baudrate(baudrate)
            self.comm.set_latency_timer(16 if baudrate <= 57600 else self.latency_timer)
            if not self.comm.open_port():
                self.get_logger().error("Failed to open")
                continue

            for servo_id in range(self.id_min, self.id_max + 1):
                if not rclpy.ok():
                    self.comm.close_port()
                    return

                self.get_logger().info(f"  Scanning ID: {servo_id}\x1b[999D\x1b[1A")
                if servo_id in found_id_set:
                    continue
                if not self.comm.try_ping(servo_id):
                    continue

                self.get_logger().info(
                    f"  ID [{servo_id}] is found, try change baudrate {self.baudrate_target}"
                )

                model_num = self.comm.try_read_uint16(dxl.ADDR_MODEL_NUMBER, servo_id)
                series = dxl.series_from_model(model_num)
                if series == dxl.SERIES_X:
                    addr_torque_enable = dxl.ADDR_X_TORQUE_ENABLE
                elif series == dxl.SERIES_P:
                    addr_torque_enable = dxl.ADDR_P_TORQUE_ENABLE
                else:
                    self.get_logger().error(f"  Unknown series {model_num}")
                    continue

                self.comm.try_write_uint8(addr_torque_enable, servo_id, dxl.TORQUE_DISABLE)
                if not self.comm.try_write_uint8(dxl.ADDR_BAUDRATE, servo_id, dyn_baudrate):
                    continue

                found_id_set.add(servo_id)
                found_ids.append(servo_id)

            self.comm.close_port()

        self.get_logger().info("                                              ")
        self.get_logger().info("=== ============ Finish scanning ============ ===\n")

        self.get_logger().info(
            f"=== Checking,  baudrate:'{self.baudrate_target}', id:[{self.id_min}]~[{self.id_max}] ==="
        )
        self.comm.set_baudrate(self.baudrate_target)
        self.comm.set_latency_timer(16 if self.baudrate_target <= 57600 else self.latency_timer)
        if not self.comm.open_port():
            self.get_logger().error(f"Failed to open USB device [{self.device_name}]")
            return

        for servo_id in found_ids:
            if self.comm.try_ping(servo_id):
                self.get_logger().info(
                    f"  ID [{servo_id}] is succeded to change baudrate {self.baudrate_target}"
                )
            else:
                self.get_logger().error(f"  ID [{servo_id}] is failed to change baudrate")

        self.comm.close_port()
        self.get_logger().info("                                              ")
        self.get_logger().info("=== ============ Finish checking ============ ===\n")


def main(args=None):
    rclpy.init(args=args)

    if dxl is None:
        node = Node("dynamixel_unify_baudrate_node")
        node.get_logger().error(
            "Python module '_mylib_dynamixel' is not available. "
            "Please rebuild dynamixel_handler."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node = DynamixelUnifyBaudrateNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
