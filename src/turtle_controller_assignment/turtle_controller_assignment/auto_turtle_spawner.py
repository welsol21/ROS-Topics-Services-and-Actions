#!/usr/bin/env python3
# coding: utf-8
"""
Task 3: Auto Turtle Spawner

- Имена: только у /turtle_name_manager/generate_unique_name (std_srvs/Trigger)
- Активные/удалённые: только из /monitor_turtles (std_srvs/Trigger)
  Формат строки: "ACTIVE:a,b;REMOVED:x,y"
- Спавн до 10 ДОПОЛНИТЕЛЬНЫХ (кроме turtle1); pen OFF; движение по кругу.
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from turtlesim.srv import Spawn, SetPen
from geometry_msgs.msg import Twist

# двигать ли также и turtle1
MOVE_TURTLE1 = True


class AutoTurtleSpawner(Node):
    def __init__(self):
        super().__init__('auto_turtle_spawner')

        # Callback groups для избежания deadlock
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.client_cb_group = MutuallyExclusiveCallbackGroup()

        # --- клиенты сервисов ---
        self.cli_name    = self.create_client(Trigger, '/turtle_name_manager/generate_unique_name', callback_group=self.client_cb_group)
        self.cli_monitor = self.create_client(Trigger, '/monitor_turtles', callback_group=self.client_cb_group)
        self.cli_spawn   = self.create_client(Spawn,   '/spawn', callback_group=self.client_cb_group)

        # дождёмся сервисов
        for cli, path in [
            (self.cli_name, '/turtle_name_manager/generate_unique_name'),
            (self.cli_monitor, '/monitor_turtles'),
            (self.cli_spawn, '/spawn'),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service: {path}')

        # --- локальные структуры ---
        self.max_additional = 10                  # лимит ДОПОЛНИТЕЛЬНЫХ
        self.my_turtles: set[str] = set()         # имена наших дополнительных
        self.cmd_pubs: dict[str, rclpy.publisher.Publisher] = {}   # name -> /<name>/cmd_vel
        self.omega: dict[str, float] = {}         # name -> угловая скорость
        if MOVE_TURTLE1:
            self.turtle1_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # --- таймеры ---
        self.manage_timer = self.create_timer(2.0, self._manage, callback_group=self.timer_cb_group)   # решение о спавне
        self.move_timer   = self.create_timer(0.1, self._move_all, callback_group=self.timer_cb_group) # движение
        self._tick = 0  # для редкого логирования из таймера движения

        self.get_logger().info('🧩 AutoTurtleSpawner started')

    # ---------- утилиты вызова Trigger ----------
    def _call_trigger(self, client, timeout_sec=2.0) -> str:
        """Синхронно вызывает Trigger сервис с таймаутом"""
        if not client.service_is_ready():
            self.get_logger().warning(f'Service {client.srv_name} not ready')
            return ''
        
        req = Trigger.Request()
        fut = client.call_async(req)
        
        # Ждем результат с таймаутом
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
        Читает монитор и возвращает (active_set, removed_set).
        Ожидаем формат: "ACTIVE:a,b;REMOVED:x,y".
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

    # ---------- основная логика ----------
    def _manage(self):
        active, removed = self._ask_monitor()
        
        self.get_logger().info(f'🔍 Monitor status - ACTIVE: {active}, REMOVED: {removed}')

        # почистим локальные структуры по REMOVED
        for name in list(self.my_turtles):
            if name in removed:
                self.get_logger().warn(f'Cleanup removed turtle reported by monitor: {name}')
                self._cleanup(name)

        # считаем количество дополнительных по данным монитора
        active_additional = len([n for n in active if n != 'turtle1'])
        self.get_logger().info(f'📊 Active additional turtles: {active_additional}/{self.max_additional}')
        
        if active_additional < self.max_additional:
            self.get_logger().info(f'🚀 Attempting to spawn new turtle...')
            self._spawn_one()
        else:
            self.get_logger().info(f'✋ Max turtles reached ({self.max_additional}), not spawning')

    def _spawn_one(self):
        name = self._get_unique_name()
        if not name:
            self.get_logger().error('❌ Failed to get unique name from name manager')
            return
        
        self.get_logger().info(f'📝 Got unique name from manager: {name}')

        # случайные безопасные параметры
        r = random.uniform(1.0, 4.0)
        phi = random.uniform(0.0, 2 * math.pi)
        cx = random.uniform(1.5, 9.5)
        cy = random.uniform(1.5, 9.5)
        x = max(0.5, min(10.5, cx + r * math.cos(phi)))
        y = max(0.5, min(10.5, cy + r * math.sin(phi)))

        direction = random.choice([-1.0, 1.0])
        theta = (phi + direction * math.pi / 2.0) % (2 * math.pi)

        # вызов /spawn
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

        # выключим ручку
        self._pen_off(spawned)

        # подготовим паблишер для движения
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
        # двигаем наших дополнительных
        for name, pub in list(self.cmd_pubs.items()):
            if name not in self.my_turtles:
                continue
            tw = Twist()
            tw.linear.x = 1.0
            tw.angular.z = float(self.omega.get(name, 0.8))
            pub.publish(tw)
            moved += 1

        # (опционально) двигаем и turtle1
        if MOVE_TURTLE1:
            tw = Twist()
            tw.linear.x = 1.0
            tw.angular.z = 1.0
            self.turtle1_pub.publish(tw)
            moved += 1

        # «пульс», чтобы видеть, что таймер жив
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
