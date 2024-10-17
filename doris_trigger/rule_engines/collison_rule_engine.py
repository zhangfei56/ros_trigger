import time

from .base_rule_engine import BaseRuleEngine

# 碰撞规则
class CollisionRuleEngine(BaseRuleEngine):
    trigger_topics = ['/odom']
    name = 'collision'

    def file_name_(self):
        # 根据时间生成文件名
        time_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        return f'collision_{time_str}.bag'
        
    # 触发条件
    def condition_start(self, trigger_msg):
        if trigger_msg.topic == '/odom':
            return True
        return False
    
    def condition_end(self, trigger_msg ):
        if time.time() - self.trigger_start_time > 10:
            return True
        return False
