#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

PIN = 29  # 示例：你要换成你实际用的物理针脚号/BOARD编号

def main():
    rospy.init_node("gpio_to_ros_gpio_lib", disable_signals=False)  # 默认即可
    active_level = rospy.get_param("~active_level", 1)
    debounce_s = rospy.get_param("~debounce_s", 0.3)
    rate_hz = rospy.get_param("~rate_hz", 50)

    pub = rospy.Publisher("/charging_state", Bool, queue_size=10)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)  

    last_raw = None
    last_change = time.time()
    stable = False

    rate = rospy.Rate(rate_hz)

    rospy.loginfo("GPIO node started. Ctrl+C to exit.")

    try:
        while not rospy.is_shutdown():
            raw = GPIO.input(PIN)

            # 仅用于观察
            rospy.loginfo_throttle(1.0, f"[GPIO] raw={raw}, stable={stable}")

            if raw != last_raw:
                last_raw = raw
                last_change = time.time()

            if time.time() - last_change >= debounce_s:
                charging = (raw == active_level)
                if charging != stable:
                    stable = charging
                    rospy.loginfo(f"[GPIO] charging_state -> {stable}")
                pub.publish(Bool(stable))

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    except KeyboardInterrupt:
        # 有时候 rospy 会先处理 SIGINT，但这里兜底保证能退
        rospy.loginfo("KeyboardInterrupt, shutting down...")
        rospy.signal_shutdown("KeyboardInterrupt")

    finally:
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up, exit.")


if __name__ == "__main__":
    main()
