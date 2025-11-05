#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose

CHECK_TIMEOUT = 2.5   # если дольше не было позы — считаем «нет сигнала»
HARD_REMOVE  = 5.0    # если нет сигнала дольше — считаем удалённой

class TurtleMonitorService(Node):
    def __init__(self):
        super().__init__('turtle_monitor_service')

        # имя -> {'last_pose': t, 'alive': bool}
        self.turtles = {}
        self.create_service(Trigger, '/monitor_turtles', self.on_monitor)
        self.create_timer(1.0, self._health_scan)
        # Таймер для сканирования топиков и обнаружения новых черепашек
        self.create_timer(2.0, self._scan_topics)

        # по умолчанию следим за turtle1
        self._ensure_subscription('turtle1')

        self.get_logger().info('🐢 Turtle Monitor Service started')

    def _ensure_subscription(self, name: str):
        if name in self.turtles:
            return
        self.turtles[name] = {'last_pose': time.time(), 'alive': False}
        self.create_subscription(Pose, f'/{name}/pose',
                                 lambda msg, n=name: self._on_pose(n),
                                 10)
        self.get_logger().info(f'📡 Now monitoring pose of: {name}')

    def _on_pose(self, name: str):
        now = time.time()
        info = self.turtles.get(name)
        if not info:
            # новое имя (если появилось внезапно)
            self._ensure_subscription(name)
            info = self.turtles[name]
        info['last_pose'] = now
        info['alive'] = True

    def _scan_topics(self):
        """Сканирует топики и добавляет подписки на новые черепашки"""
        topic_list = self.get_topic_names_and_types()
        
        # Собираем список активных черепашек из топиков
        active_topics = set()
        for topic_name, _ in topic_list:
            # Ищем топики вида /<name>/pose
            if topic_name.endswith('/pose'):
                # Извлекаем имя черепашки
                turtle_name = topic_name.split('/')[1]
                if turtle_name:
                    active_topics.add(turtle_name)
                    # Проверяем, что это не уже отслеживаемая черепашка
                    if turtle_name not in self.turtles:
                        self._ensure_subscription(turtle_name)
        
        # Удаляем из отслеживания черепашек, топики которых исчезли
        for turtle_name in list(self.turtles.keys()):
            if turtle_name not in active_topics:
                if self.turtles[turtle_name]['alive']:
                    self.get_logger().warning(f'🗑️  Topic disappeared, removing turtle from tracking: {turtle_name}')
                # Полностью удаляем из отслеживания
                del self.turtles[turtle_name]

    def _health_scan(self):
        now = time.time()
        for name, info in list(self.turtles.items()):
            dt = now - info['last_pose']
            if dt > HARD_REMOVE:
                # считаем удалённой (не приходила поза очень давно)
                if info['alive']:
                    self.get_logger().warning(f'⚠️  Turtle appears removed: {name}')
                info['alive'] = False
            elif dt > CHECK_TIMEOUT:
                # временно «нет сигнала», но окончательно не удаляем
                info['alive'] = False

    def on_monitor(self, request, response):
        # ЗДЕСЬ не сканируем топики — монитор только «отвечает» текущим статусом.
        alive = [n for n, d in self.turtles.items() if d['alive']]
        removed = [n for n, d in self.turtles.items() if not d['alive']]

        # строка вида: "ACTIVE:a,b;REMOVED:x,y"
        response.success = True
        response.message = f"ACTIVE:{','.join(alive)};REMOVED:{','.join(removed)}"
        return response

def main():
    rclpy.init()
    node = TurtleMonitorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
