
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Int16MultiArray
from dataclasses import dataclass
import time
import serial
import json
import rclpy

@dataclass
class SensorReading:
    id: int
    type: str
    value: float
    ts: int

    @staticmethod
    def from_json(data) -> "SensorReading":
        return SensorReading(
            id=data.get("id"),
            type=data.get("type"),
            value=data.get("value"),
            ts=data.get("ts", int(time.time()))
        )
    
    def __str__(self):
        return (f"SensorReading(id={self.id}, type={self.type}, "
                f"value={self.value}, ts={self.ts})")

class ToFArrayNode(Node):
    def __init__(self):
        super().__init__('tof_node')
    
        

        try:
            self.ser = self._initialize_tof_array()
        except Exception as e:
            self.ser = None
            self.get_logger().error(f"Failed to initialize ToF sensor: {str(e)}")
            raise e
        
        self.publisher_ = self.create_publisher(Int16MultiArray, 'tof', 10)
        self.timer = self.create_timer(0.1, self._read_tof_data)  # Read every 100ms
        
    def _initialize_tof_array(self):
        self.get_logger().info("Initializing TOF Array")
        
        ser = serial.Serial(
            port='/dev/mypico',
            baudrate=115200,  # Adjust if your Pico uses different baud rate
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1,  # Reduced timeout for faster response
            write_timeout=0.1
        )
        # Flush any existing data in buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self.get_logger().info("TOF Array initialized")
        return ser

    def _read_tof_data(self):
        if not rclpy.ok():
            self.get_logger().info("ROS is shutting down, stopping ToF data reading")
            return
        if not self.ser or not self.ser.is_open:
            self.get_logger().warning("ToF sensor serial connection is not open")
            return
        
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    data = json.loads(line)
                    reading = SensorReading.from_json(data)                    

                    msg = Int16MultiArray()
                    msg.data = [reading.id, int(reading.value)]
                    self.publisher_.publish(msg)
                    self.get_logger().debug(f'Published -> ID: {reading.id}, Value: {reading.value}')
                      
            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode error: {e}")
            except UnicodeDecodeError as e:
                self.get_logger().error(f"Unicode decode error: {e}")
    
    def destroy_node(self):
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.publisher_)
        if self.ser and self.ser.is_open:
            # print(), not get_logger(): on Ctrl-C the rclpy context is already
            # torn down, so rosout publishing fails with "publisher's context is invalid".
            print("Closing ToF sensor serial connection")
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ToFArrayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()