#!/usr/bin/env python3
"""
Task 3: Auto Turtle Spawner - spawns bots periodically and moves them in circles
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from turtlesim.srv import Spawn, SetPen
from geometry_msgs.msg import Twist

# Whether to also move turtle1 (disabled to avoid conflict)
MOVE_TURTLE1 = False


class AutoTurtleSpawner(Node):
    """Node: spawns bots periodically and moves them in circles"""
    def __init__(self):
        super().__init__('auto_turtle_spawner')

        # Callback groups to avoid deadlocks
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Flag to enable/disable spawning (for collection action)
        self.spawning_enabled = True

        # Service clients
        self.cli_name    = self.create_client(Trigger, '/turtle_name_manager/generate_unique_name', callback_group=self.client_cb_group)
        self.cli_monitor = self.create_client(Trigger, '/monitor_turtles', callback_group=self.client_cb_group)
        self.cli_spawn   = self.create_client(Spawn,   '/spawn', callback_group=self.client_cb_group)
        
        # Service to enable/disable spawning
        self.enable_service = self.create_service(
            Trigger,
            '/spawner/enable',
            self.enable_callback,
            callback_group=self.client_cb_group
        )
        self.disable_service = self.create_service(
            Trigger,
            '/spawner/disable',
            self.disable_callback,
            callback_group=self.client_cb_group
        )

        # Wait for services
        for cli, path in [
            (self.cli_name, '/turtle_name_manager/generate_unique_name'),
            (self.cli_monitor, '/monitor_turtles'),
            (self.cli_spawn, '/spawn'),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service: {path}')

        # Local structures
        self.max_additional = 10                  # limit of additional turtles (excluding turtle1)
        self.my_turtles: set[str] = set()         # names of our additional turtles
        self.cmd_pubs: dict[str, Publisher] = {}   # name -> /<name>/cmd_vel
        self.omega: dict[str, float] = {}         # name -> angular velocity
        if MOVE_TURTLE1:
            self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timers
        self.spawn_timer = self.create_timer(5.0, self._try_spawn, callback_group=self.timer_cb_group)  # spawn every 5 seconds
        self.move_timer   = self.create_timer(0.1, self._move_all, callback_group=self.timer_cb_group) # movement
        self._tick = 0  # для редкого логирования из таймера движения

        self.get_logger().info('🧩 AutoTurtleSpawner started')
    
    def enable_callback(self, request, response):
        """Enable spawning and movement"""
        self.spawning_enabled = True
        response.success = True
        response.message = 'Auto spawner enabled'
        self.get_logger().info('✅ Auto spawner enabled')
        return response
    
    def disable_callback(self, request, response):
        """Disable spawning and movement"""
        self.spawning_enabled = False
        response.success = True
        response.message = 'Auto spawner disabled'
        self.get_logger().info('⏸️  Auto spawner disabled')
        return response

    # ---------- Trigger call utilities ----------
    def _call_trigger(self, client, timeout_sec=2.0) -> str:
        """Synchronously call Trigger service with timeout"""
        if not client.service_is_ready():
            self.get_logger().warning(f'Service {client.srv_name} not ready')
            return ''
        
        req = Trigger.Request()
        fut = client.call_async(req)
        
        # Wait for result with timeout
        start = time.time()
        while not fut.done() and (time.time() - start) < timeout_sec:
            time.sleep(0.01)
        
        if fut.done():
            res = fut.result()
            if res and res.success:
                return res.message or ''
        else:
            self.get_logger().warning(f'Service call timed out: {client.srv_name}')
        
        return ''

    def _get_unique_name(self) -> str:
        name = (self._call_trigger(self.cli_name) or '').strip()
        if not name:
            self.get_logger().error('Name manager returned empty name')
        return name

    def _ask_monitor(self) -> tuple[set[str], set[str]]:
        """
        Read monitor and return (active_set, removed_set).
        Expected format: "ACTIVE:a,b;REMOVED:x,y".
        """
        raw = self._call_trigger(self.cli_monitor)
        active, removed = set(), set()
        if raw:
            for part in [p.strip() for p in raw.split(';') if p.strip()]:
                if part.startswith('ACTIVE:'):
                    payload = part[len('ACTIVE:'):]
                    active = set([x for x in payload.split(',') if x])
                elif part.startswith('REMOVED:'):
                    payload = part[len('REMOVED:'):]
                    removed = set([x for x in payload.split(',') if x])
        return active, removed

    # ---------- Main logic ----------
    def _try_spawn(self):
        """Called every 5 seconds to attempt to spawn a turtle"""
        # Skip if spawning is disabled
        if not self.spawning_enabled:
            return
        
        active, removed = self._ask_monitor()
        
        self.get_logger().info(f'🔍 Monitor status - ACTIVE: {active}, REMOVED: {removed}')

        # clean local structures for REMOVED
        for name in list(self.my_turtles):
            if name in removed:
                self.get_logger().warn(f'Cleanup removed turtle reported by monitor: {name}')
                self._cleanup(name)

        # count additional turtles (excluding turtle1)
        additional_active = len([n for n in active if n != 'turtle1'])
        self.get_logger().info(f'📊 Additional turtles: {additional_active}/{self.max_additional} (total: {len(active)})')
        
        if additional_active < self.max_additional:
            self.get_logger().info(f'🚀 Spawning new turtle...')
            self._spawn_one()
        else:
            self.get_logger().info(f'✋ Max additional turtles reached ({self.max_additional}), not spawning')

    def _spawn_one(self):
        name = self._get_unique_name()
        if not name:
            self.get_logger().error('❌ Failed to get unique name from name manager')
            return
        
        self.get_logger().info(f'📝 Got unique name from manager: {name}')

        # Generate coordinates per assignment formula
        # r ~ U(1, 5)
        r = random.uniform(1.0, 5.0)
        # φ ~ U(0, 2π)
        phi = random.uniform(0.0, 2.0 * math.pi)
        # x' ~ U(-5.5+r, 5.5-r)
        x_prime = random.uniform(-5.5 + r, 5.5 - r)
        # y' ~ U(-5.5+r, 5.5-r)
        y_prime = random.uniform(-5.5 + r, 5.5 - r)
        # x = 5.5 + x' + r*cos(φ)
        x = 5.5 + x_prime + r * math.cos(phi)
        # y = 5.5 + y' + r*sin(φ)
        y = 5.5 + y_prime + r * math.sin(phi)
        # θ = φ ± π/2 (random sign)
        direction = random.choice([-1.0, 1.0])
        theta = phi + direction * math.pi / 2.0

        # call /spawn
        if not self.cli_spawn.service_is_ready():
            self.get_logger().warning('Spawn service not ready')
            return
        
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = name

        fut = self.cli_spawn.call_async(req)
        
        # Ждем результат с таймаутом
        start = time.time()
        while not fut.done() and (time.time() - start) < 2.0:
            time.sleep(0.01)
        
        if not fut.done():
            self.get_logger().error(f'Spawn service timeout for {name}')
            return
        
        res = fut.result()
        if not res:
            self.get_logger().error(f'Failed to spawn {name}')
            return

        spawned = res.name
        self.get_logger().info(f'✅ Spawned: {spawned}')
        self.my_turtles.add(spawned)

        # turn pen off
        self._pen_off(spawned)

        # prepare publisher for movement
        self.cmd_pubs[spawned] = self.create_publisher(Twist, f'/{spawned}/cmd_vel', 10)
        self.omega[spawned] = direction * (1.0 / r)

    def _pen_off(self, name: str):
        try:
            cli = self.create_client(SetPen, f'/{name}/set_pen')
            if not cli.wait_for_service(timeout_sec=1.0):
                return
            req = SetPen.Request()
            req.r = 0
            req.g = 0
            req.b = 0
            req.width = 1
            req.off = True
            cli.call_async(req)  # без ожидания
        except Exception:
            pass

    def _move_all(self):
        moved = 0
        # move our additional turtles
        for name, pub in list(self.cmd_pubs.items()):
            if name not in self.my_turtles:
                continue
            tw = Twist()
            tw.linear.x = 1.0
            tw.angular.z = float(self.omega.get(name, 0.8))
            pub.publish(tw)
            moved += 1

        # optionally move turtle1
        if MOVE_TURTLE1:
            tw = Twist()
            tw.linear.x = 1.0
            tw.angular.z = 1.0
            self.turtle1_pub.publish(tw)
            moved += 1

        # heartbeat to show timer is active
        self._tick += 1
        if self._tick % 10 == 0:
            self.get_logger().info(f'🟢 move-timer tick, publishing to {moved} turtle(s)')

    def _cleanup(self, name: str):
        self.my_turtles.discard(name)
        self.cmd_pubs.pop(name, None)
        self.omega.pop(name, None)


def main():
    rclpy.init()
    node = AutoTurtleSpawner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
