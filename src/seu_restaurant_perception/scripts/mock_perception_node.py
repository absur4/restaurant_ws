#!/usr/bin/env python3
import json

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from seu_restaurant_msgs.msg import CustomerCall


def build_customer_call(event_cfg, frame_id, seq):
    now = rospy.Time.now()
    msg = CustomerCall()
    msg.header.stamp = now
    msg.header.frame_id = frame_id
    msg.customer_id = str(event_cfg.get("customer_id", ""))
    msg.timestamp = now
    msg.source = "wave"
    msg.table_id = str(event_cfg.get("table_id", ""))
    msg.position = PoseStamped()
    msg.position.header.stamp = now
    msg.position.header.frame_id = str(event_cfg.get("frame_id", frame_id))
    position_cfg = event_cfg.get("position", {})
    msg.position.pose.position.x = float(position_cfg.get("x", 0.0))
    msg.position.pose.position.y = float(position_cfg.get("y", 0.0))
    msg.position.pose.position.z = float(position_cfg.get("z", 0.0))
    orientation_cfg = event_cfg.get("orientation", {})
    msg.position.pose.orientation.x = float(orientation_cfg.get("x", 0.0))
    msg.position.pose.orientation.y = float(orientation_cfg.get("y", 0.0))
    msg.position.pose.orientation.z = float(orientation_cfg.get("z", 0.0))
    msg.position.pose.orientation.w = float(orientation_cfg.get("w", 1.0))
    msg.confidence = float(event_cfg.get("confidence", 1.0))
    msg.requires_assistance = bool(event_cfg.get("requires_assistance", True))
    if not msg.customer_id:
        msg.customer_id = "mock_customer_{:02d}".format(seq)
    return msg


def main():
    rospy.init_node("mock_perception_node")

    perception_ns = "/restaurant/perception"
    wave_source_enabled = bool(rospy.get_param(perception_ns + "/wave_source_enabled", True))
    voice_call_enabled = bool(rospy.get_param(perception_ns + "/voice_call_enabled", False))
    customer_call_topic = rospy.get_param(perception_ns + "/customer_call_topic", "/restaurant/customer_call")
    wave_event_topic = rospy.get_param(perception_ns + "/wave_event_topic", "/restaurant/perception/wave_event")
    debug_topic = rospy.get_param(perception_ns + "/customer_memory_debug_topic", "/restaurant/customer_memory_debug")
    frame_id = rospy.get_param(perception_ns + "/position_frame", "map")
    min_wave_confidence = float(rospy.get_param(perception_ns + "/min_wave_confidence", 0.6))
    mock_customers = rospy.get_param(perception_ns + "/mock_customers", [])

    wave_pub = rospy.Publisher(wave_event_topic, CustomerCall, queue_size=20)
    customer_call_pub = rospy.Publisher(customer_call_topic, CustomerCall, queue_size=20)
    debug_pub = rospy.Publisher(debug_topic, String, queue_size=20, latch=True)

    rospy.sleep(0.2)
    rospy.loginfo(
        "[mock_perception_node] wave_source_enabled=%s voice_call_enabled=%s mock_customers=%d",
        wave_source_enabled,
        voice_call_enabled,
        len(mock_customers),
    )

    if not wave_source_enabled:
        debug_pub.publish(String(data=json.dumps({"wave_source_enabled": False, "published_events": 0}, ensure_ascii=True)))
        rospy.spin()
        return

    start_time = rospy.Time.now()
    published_count = 0
    for seq, customer_cfg in enumerate(sorted(mock_customers, key=lambda item: float(item.get("trigger_time_sec", 0.0))), start=1):
        trigger_time_sec = float(customer_cfg.get("trigger_time_sec", 0.0))
        confidence = float(customer_cfg.get("confidence", 1.0))
        if confidence < min_wave_confidence:
            rospy.logwarn(
                "[mock_perception_node] Skip %s because confidence %.3f < min_wave_confidence %.3f",
                customer_cfg.get("customer_id", "unknown"),
                confidence,
                min_wave_confidence,
            )
            continue

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= trigger_time_sec:
                break
            rospy.sleep(0.02)

        msg = build_customer_call(customer_cfg, frame_id, seq)
        wave_pub.publish(msg)
        customer_call_pub.publish(msg)
        published_count += 1
        rospy.loginfo(
            "[mock_perception_node] Published wave event customer_id=%s trigger=%.2fs frame=%s pos=(%.2f, %.2f, %.2f) confidence=%.2f",
            msg.customer_id,
            trigger_time_sec,
            msg.position.header.frame_id,
            msg.position.pose.position.x,
            msg.position.pose.position.y,
            msg.position.pose.position.z,
            msg.confidence,
        )

    debug_pub.publish(
        String(
            data=json.dumps(
                {
                    "wave_source_enabled": wave_source_enabled,
                    "voice_call_enabled": voice_call_enabled,
                    "published_events": published_count,
                    "customer_call_topic": customer_call_topic,
                    "wave_event_topic": wave_event_topic,
                },
                ensure_ascii=True,
                sort_keys=True,
            )
        )
    )
    rospy.spin()


if __name__ == "__main__":
    main()
