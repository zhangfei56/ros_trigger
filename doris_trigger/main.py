import json
import os
import signal
import sys
import threading
import time
from data_manager import DataManager
from log_config import set_log
from rule_engines.collison_rule_engine import CollisionRuleEngine
from rule_engines.immediate_rule_engine import ImmediateRuleEngine
import rospy

from loguru import logger



rule_name_engine_cls_dict = {
    'collision': CollisionRuleEngine,
    'immediate': ImmediateRuleEngine
}

def main(**kargs):
    log_dir = kargs.get('log_dir')
    log_level = kargs.get('log_level')
    use_buffer = kargs.get('use_buffer')
    logger.info(
        f"start ros_trigger, log_dir: {log_dir}, log_level: {log_level}, use_buffer: {use_buffer}")
    set_log(log_level, log_dir)
    data_manager = DataManager(use_buffer=use_buffer)

    # 1. 获取云端规则
    # 2. 获取本地规则
    # 3. 比较规则，如果云端规则与本地规则不一致，则更新本地规则
    # 4. 执行本地规则
    

    config_json = kargs.get('config_json')
    logger.info(f"config_json: \n {config_json}")
    config_rule_engines = config_json.get("rule_engines")
    # 遍历 config_rule_engines，根据 rule_name 获取对应的 RuleEngine 类
    rule_engine_dict = {}
    for config_engine in config_rule_engines:
        engine_name = config_engine.get("name")
        engine_cls = rule_name_engine_cls_dict.get(engine_name)
        topics = config_engine.get("topics")
        rule_engine_dict[engine_name] = engine_cls(topics, data_manager)
    
    
    if kargs.get('immediate'):
        immediate_instance = rule_engine_dict.get('immediate')
        rule_engines = [immediate_instance]
    else:
        config_run_engines = config_json.get("run_engines")
        rule_engines = []
        for config_run_engine in config_run_engines:
            engine_name = config_run_engine.get("type")
            rule_engines.append(rule_engine_dict.get(engine_name))
    logger.info(f'rule engines: { rule_engines }')
    
    rospy.init_node('trigger_data_collector', anonymous=False)


    threadings = []
    # 6. 处理
    for engine in rule_engines:
        engine_thread = threading.Thread(target=engine.run)
        engine_thread.daemon = True  # Set as daemon thread to exit when main thread exits
        engine_thread.setName( "Thread-" + engine.name)
        engine_thread.start()
        logger.info(f'start thread {engine_thread.name}')
        threadings.append(engine_thread)
    
    for rule_engine in rule_engines:
        data_manager.add_engine(rule_engine)


    # 注册信号处理程序
    def signal_handler(sig, frame):
        logger.info('You pressed Ctrl+C!')
        for engine in rule_engines:
            engine.stop()
        
        # 在这里可以添加清理代码
        # sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    
    
        
    # logger.info('All threads finished')
    def check_all_thread_active():
        for engine_thread in threadings:
            if engine_thread.is_alive():
                return True
        return False
    
    try:
        while not rospy.core.is_shutdown():
            if not check_all_thread_active():
                break
            
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        logger.info("keyboard interrupt, shutting down")
        rospy.core.signal_shutdown('keyboard interrupt')
    logger.info("doris trigger exit")

    
if __name__ == '__main__':
    try:
        config_path = os.path.join(os.path.dirname(__file__),'..', 'default_rule_engine.json')
        with open(config_path, 'r') as f:
            config_json = json.load(f)
        
        main(use_buffer=False, immediate=True, config_json=config_json, log_level='debug', log_dir='./logs')
        

    except Exception as e:
        print(f"An error occurred: {e}")
