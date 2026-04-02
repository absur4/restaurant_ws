import copy
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped

from seu_restaurant_msgs.msg import CustomerCall


STATUS_WAITING = "waiting"
STATUS_SELECTED = "selected"
STATUS_SERVING = "serving"
STATUS_SERVED = "served"
STATUS_EXPIRED = "expired"


@dataclass
class CustomerMemoryEntry:
    customer_id: str
    source: str
    table_id: str = ""
    position: Optional[PoseStamped] = None
    first_seen: rospy.Time = field(default_factory=rospy.Time.now)
    last_seen: rospy.Time = field(default_factory=rospy.Time.now)
    confidence: float = 0.0
    status: str = STATUS_WAITING
    event_count: int = 1
    selected_at: Optional[rospy.Time] = None
    serving_at: Optional[rospy.Time] = None
    served_at: Optional[rospy.Time] = None

    def clone_position(self):
        return copy.deepcopy(self.position)


class CustomerMemoryManager:
    def __init__(self, config=None):
        config = config or {}
        self.memory_expire_sec = float(config.get("memory_expire_sec", 20.0))
        self.duplicate_distance_thresh = float(config.get("duplicate_distance_thresh", 0.6))
        self.duplicate_time_thresh = float(config.get("duplicate_time_thresh", 2.0))
        self.selection_policy = str(config.get("selection_policy", "fifo")).lower()
        self.min_wave_confidence = float(config.get("min_wave_confidence", 0.5))
        self._entries: Dict[str, CustomerMemoryEntry] = {}
        self._id_counter = 0

    def add_or_update_call(self, call_msg):
        if call_msg is None:
            return None

        self.expire_old_entries()
        event_time = self._event_time(call_msg)
        if float(call_msg.confidence) < self.min_wave_confidence:
            return None

        matched_entry = self._find_duplicate(call_msg, event_time, include_served=True)
        if matched_entry and matched_entry.status == STATUS_SERVED:
            return matched_entry

        if matched_entry:
            matched_entry.last_seen = event_time
            matched_entry.confidence = max(matched_entry.confidence, float(call_msg.confidence))
            matched_entry.position = copy.deepcopy(call_msg.position)
            matched_entry.table_id = call_msg.table_id or matched_entry.table_id
            matched_entry.source = call_msg.source or matched_entry.source
            matched_entry.event_count += 1
            return matched_entry

        customer_id = call_msg.customer_id or self._generate_customer_id()
        if customer_id in self._entries and self._entries[customer_id].status == STATUS_SERVED:
            return self._entries[customer_id]

        entry = CustomerMemoryEntry(
            customer_id=customer_id,
            source=call_msg.source or "wave",
            table_id=call_msg.table_id,
            position=copy.deepcopy(call_msg.position),
            first_seen=event_time,
            last_seen=event_time,
            confidence=float(call_msg.confidence),
            status=STATUS_WAITING,
            event_count=1,
        )
        self._entries[customer_id] = entry
        return entry

    def get_next_customer(self):
        self.expire_old_entries()
        waiting = self.get_waiting_customers()
        if not waiting:
            return None
        return waiting[0]

    def mark_selected(self, customer_id):
        entry = self._entries.get(customer_id)
        if entry:
            entry.status = STATUS_SELECTED
            entry.selected_at = rospy.Time.now()
        return entry

    def mark_serving(self, customer_id):
        entry = self._entries.get(customer_id)
        if entry:
            entry.status = STATUS_SERVING
            entry.serving_at = rospy.Time.now()
        return entry

    def mark_served(self, customer_id):
        entry = self._entries.get(customer_id)
        if entry:
            entry.status = STATUS_SERVED
            entry.served_at = rospy.Time.now()
        return entry

    def expire_old_entries(self):
        now = rospy.Time.now()
        for entry in self._entries.values():
            if entry.status == STATUS_SERVED:
                continue
            age_sec = max(0.0, (now - entry.last_seen).to_sec())
            if age_sec > self.memory_expire_sec:
                entry.status = STATUS_EXPIRED

    def get_waiting_customers(self):
        waiting = [entry for entry in self._entries.values() if entry.status == STATUS_WAITING]
        if self.selection_policy == "fifo":
            return sorted(waiting, key=lambda entry: (entry.first_seen.to_sec(), -entry.confidence, entry.customer_id))
        return sorted(waiting, key=lambda entry: (entry.first_seen.to_sec(), -entry.confidence, entry.customer_id))

    def get_entry(self, customer_id):
        return self._entries.get(customer_id)

    def has_waiting_customers(self):
        return bool(self.get_waiting_customers())

    def to_debug_dict(self):
        waiting_customers = self.get_waiting_customers()
        waiting_ids = [entry.customer_id for entry in waiting_customers]
        queue_index_map = {customer_id: index for index, customer_id in enumerate(waiting_ids)}
        selected_id = ""
        serving_id = ""
        served_ids = []
        entries = []
        for entry in sorted(self._entries.values(), key=lambda item: (item.first_seen.to_sec(), item.customer_id)):
            if entry.status == STATUS_SELECTED and not selected_id:
                selected_id = entry.customer_id
            if entry.status == STATUS_SERVING and not serving_id:
                serving_id = entry.customer_id
            if entry.status == STATUS_SERVED:
                served_ids.append(entry.customer_id)
            entries.append(
                {
                    "customer_id": entry.customer_id,
                    "source": entry.source,
                    "table_id": entry.table_id,
                    "status": entry.status,
                    "confidence": round(entry.confidence, 3),
                    "event_count": entry.event_count,
                    "first_seen": round(entry.first_seen.to_sec(), 3),
                    "last_seen": round(entry.last_seen.to_sec(), 3),
                    "queue_index": queue_index_map.get(entry.customer_id, -1),
                    "position": self._pose_to_dict(entry.position),
                }
            )
        return {
            "selection_policy": self.selection_policy,
            "memory_expire_sec": self.memory_expire_sec,
            "duplicate_distance_thresh": self.duplicate_distance_thresh,
            "duplicate_time_thresh": self.duplicate_time_thresh,
            "waiting_ids": waiting_ids,
            "selected_id": selected_id,
            "serving_id": serving_id,
            "served_ids": served_ids,
            "queue_order": waiting_ids,
            "waiting_count": len(waiting_ids),
            "serving_count": 1 if serving_id else 0,
            "served_count": len(served_ids),
            "entries": entries,
        }

    def _find_duplicate(self, call_msg, event_time, include_served=False):
        best_entry = None
        best_distance = None
        for entry in self._entries.values():
            if entry.status == STATUS_EXPIRED:
                continue
            if not include_served and entry.status == STATUS_SERVED:
                continue
            if call_msg.customer_id and entry.customer_id == call_msg.customer_id:
                return entry
            if not self._is_within_duplicate_window(entry, event_time):
                continue
            distance = self._distance_between(entry.position, call_msg.position)
            if distance is None or distance > self.duplicate_distance_thresh:
                continue
            if best_distance is None or distance < best_distance:
                best_entry = entry
                best_distance = distance
        return best_entry

    def _is_within_duplicate_window(self, entry, event_time):
        return abs((event_time - entry.last_seen).to_sec()) <= self.duplicate_time_thresh

    def _generate_customer_id(self):
        self._id_counter += 1
        return "customer_{:03d}".format(self._id_counter)

    def _event_time(self, call_msg):
        if getattr(call_msg, "timestamp", rospy.Time()).to_sec() > 0.0:
            return call_msg.timestamp
        if getattr(call_msg.header, "stamp", rospy.Time()).to_sec() > 0.0:
            return call_msg.header.stamp
        return rospy.Time.now()

    def _distance_between(self, pose_a, pose_b):
        if pose_a is None or pose_b is None:
            return None
        if pose_a.header.frame_id and pose_b.header.frame_id and pose_a.header.frame_id != pose_b.header.frame_id:
            return None
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        dz = pose_a.pose.position.z - pose_b.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _pose_to_dict(self, pose):
        if pose is None:
            return {}
        return {
            "frame_id": pose.header.frame_id,
            "x": round(pose.pose.position.x, 3),
            "y": round(pose.pose.position.y, 3),
            "z": round(pose.pose.position.z, 3),
        }


def resolve_customer_table(entry, available_tables, navigation_targets):
    if entry is None:
        return "", ""

    if entry.table_id:
        table_cfg = available_tables.get(entry.table_id, {})
        return entry.table_id, table_cfg.get("pose_name", entry.table_id)

    position = entry.position
    if position is None:
        return "", ""

    best_table_id = ""
    best_pose_name = ""
    best_distance = None
    for table_id, table_cfg in available_tables.items():
        pose_name = table_cfg.get("pose_name", "")
        target_cfg = navigation_targets.get(pose_name, {})
        target_position = target_cfg.get("position")
        if not target_position:
            continue
        dx = float(target_position.get("x", 0.0)) - position.pose.position.x
        dy = float(target_position.get("y", 0.0)) - position.pose.position.y
        dz = float(target_position.get("z", 0.0)) - position.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_table_id = table_id
            best_pose_name = pose_name

    return best_table_id, best_pose_name
