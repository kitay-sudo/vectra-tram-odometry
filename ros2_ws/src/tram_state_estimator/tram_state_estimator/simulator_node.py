"""Нода-имитатор трамвая: публикует ручку и скорости колёс.

Нужна для запуска всего стенда одной командой без реального подвижного
состава. Использует ту же модель, что и прогон сценариев, и дополнительно
публикует истинную скорость в `tram/truth` - только для оценки качества,
оценщик её не видит.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

from .plant import Plant, Track

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

DT = 0.001


class SimulatorNode(Node):
    def __init__(self):
        super().__init__("tram_simulator")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("mu", 0.25)
        self.declare_parameter("grade", 0.0)
        self.declare_parameter("seed", 0)

        rate = float(self.get_parameter("rate_hz").value)
        mu = float(self.get_parameter("mu").value)
        grade = float(self.get_parameter("grade").value)
        seed = int(self.get_parameter("seed").value)

        track = Track(grade=lambda s: grade, mu=lambda s, t: mu)
        self.plant = Plant(track, dt=DT, seed=seed)
        self.sub_steps = int(round(1.0 / rate / DT))

        self.pub_wheels = self.create_publisher(
            JointState, "tram/wheel_speeds", SENSOR_QOS)
        self.pub_handle = self.create_publisher(Int8, "tram/handle", SENSOR_QOS)
        self.pub_truth = self.create_publisher(
            TwistStamped, "tram/truth", SENSOR_QOS)

        self.names = [f"wheel_{i}" for i in range(self.plant.nw)]
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"имитатор запущен: mu={mu}, уклон={grade}")

    def _driver(self, t: float) -> int:
        """Разгон, выбег, торможение до остановки, повтор."""
        c = t % 45.0
        if c < 18.0:
            return 4
        if c < 28.0:
            return 0
        return -3

    def _tick(self):
        notch = self._driver(self.plant.t)
        for _ in range(self.sub_steps):
            self.plant.step(notch)
        meas = self.plant.measure()

        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = self.names
        js.velocity = [float(x) for x in meas]
        self.pub_wheels.publish(js)

        self.pub_handle.publish(Int8(data=int(notch)))

        tw = TwistStamped()
        tw.header.stamp = stamp
        tw.twist.linear.x = float(self.plant.v)
        self.pub_truth.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
