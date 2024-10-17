import time

from .base_rule_engine import BaseRuleEngine

# Immediate
class ImmediateRuleEngine(BaseRuleEngine):
    trigger_topics = []
    topics = [ '/odom']
    name = 'immediate'

    def file_name_(self):
        # 根据时间生成文件名
        time_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        return f'immediate_{time_str}.bag'
        
    # 触发条件
    def condition_start(self, topic_msg):
        return True
    
    # 手动cancel 结束
    def condition_end(self, topic_msg):
        return False
