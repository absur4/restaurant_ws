from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PoseStamped

from seu_restaurant_msgs.msg import OrderInfo as OrderInfoMsg
from seu_restaurant_msgs.msg import OrderItem as OrderItemMsg


@dataclass
class Customer:
    customer_id: str = ""
    table_id: str = ""
    pose_name: str = ""
    notes: str = ""
    call_source: str = "mock"
    confidence: float = 0.0
    position: Optional[PoseStamped] = None
    target_pose: Optional[PoseStamped] = None
    # perception writes: call_source, confidence
    # perception/customer memory writes: position, target_pose
    # smach writes: customer_id, table_id, pose_name


@dataclass
class OrderItem:
    item_id: str = ""
    display_name: str = ""
    quantity: int = 1
    notes: str = ""

    @classmethod
    def from_msg(cls, msg):
        return cls(
            item_id=msg.item_id,
            display_name=msg.display_name,
            quantity=msg.quantity,
            notes=msg.notes,
        )

    def to_msg(self):
        msg = OrderItemMsg()
        msg.item_id = self.item_id
        msg.display_name = self.display_name
        msg.quantity = self.quantity
        msg.notes = self.notes
        return msg


@dataclass
class Order:
    order_id: str = ""
    customer_id: str = ""
    table_id: str = ""
    items: List[OrderItem] = field(default_factory=list)
    special_requests: str = ""
    confirmed: bool = False
    raw_text: str = ""
    # hri writes raw_text
    # llm writes items, special_requests
    # smach writes order_id/customer_id/table_id/confirmed

    @classmethod
    def from_msg(cls, msg):
        return cls(
            order_id=msg.order_id,
            customer_id=msg.customer_id,
            table_id=msg.table_id,
            items=[OrderItem.from_msg(item) for item in msg.items],
            special_requests=msg.special_requests,
            confirmed=msg.confirmed,
        )

    def to_msg(self):
        msg = OrderInfoMsg()
        msg.order_id = self.order_id
        msg.customer_id = self.customer_id
        msg.table_id = self.table_id
        msg.items = [item.to_msg() for item in self.items]
        msg.special_requests = self.special_requests
        msg.confirmed = self.confirmed
        return msg


@dataclass
class TaskContext:
    mission_id: str = "restaurant_demo"
    cycle_index: int = 0
    max_cycles: int = 1
    status: str = "created"
    selected_customer: Customer = field(default_factory=Customer)
    current_customer: Customer = field(default_factory=Customer)
    current_order: Order = field(default_factory=Order)
    available_tables: Dict[str, dict] = field(default_factory=dict)
    menu: Dict[str, dict] = field(default_factory=dict)
    navigation_targets: Dict[str, dict] = field(default_factory=dict)
    service_names: Dict[str, dict] = field(default_factory=dict)
    bar_pose_name: str = "bar_default"
    last_heard_text: str = ""
    last_error: str = ""
    debug_notes: List[str] = field(default_factory=list)
    should_finish: bool = False
    customer_memory: Any = None

    def note(self, message):
        self.debug_notes.append(message)
