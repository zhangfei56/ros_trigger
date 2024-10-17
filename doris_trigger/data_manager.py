from collections import deque
import threading
from nav_msgs.msg import Odometry
from loguru import logger
from rospy.msg import AnyMsg
from rule_engines.base_rule_engine import BaseRuleEngine
import time
import rospy

topic_type_map = {
    '/odom' : Odometry,
}


def get_topic_type(topic):
    topic_type = topic_type_map.get(topic)
    if topic_type is None:
        topic_type = AnyMsg
    return topic_type



class TopicMessage:
    def __init__(self, topic, type, msg):
        self.topic = topic
        self.type = type
        self.msg = msg
        self.receive_time = time.time()
        
class DataManager():
    def __init__(self, use_buffer):
        self.trigger_topic_engine_dict = {}
        self.time_threshold = 15 # 15s
        self.use_buffer = use_buffer
        if self.use_buffer:
            self.topic_data_buffer = deque(maxlen=30*self.time_threshold*10 ) # 15s * 30hz * 10 topic
        
        self.raw_topic_engine_dict = {}
        self.thread_lock = threading.Lock()
        self.subscribe_topic_dict = {}

    def subscribe_topic(self, topic):
        if topic in self.subscribe_topic_dict:
            return
        def callback(msg):
            self.receive_ros_data(TopicMessage(topic, get_topic_type(topic), msg))
        self.subscribe_topic_dict[topic] = rospy.Subscriber(topic, get_topic_type(topic), callback)
        logger.info(f'subscribe topic: {topic}')

    def subscribe_topics(self, topics):
        for topic in topics:
            self.subscribe_topic(topic)

    def receive_ros_data(self, topic_msg: TopicMessage):
        # enqueue trigger topic data
        logger.debug(f'receive ros data { topic_msg.topic}')
        trigger_engines = self.trigger_topic_engine_dict.get(topic_msg.topic)
        if trigger_engines is not None:
            for engine in trigger_engines:
                engine.trigger_topic_queue.put(topic_msg)
        
        # insert msg to queue
        raw_engines = self.raw_topic_engine_dict.get(topic_msg.topic)
        if raw_engines is not None:
            for engine in raw_engines:
                engine.raw_topic_queue.put(topic_msg)
                
        # save to buffer
        if self.use_buffer:
            # TODO 测试获取不到锁时，数据的 receive time
            self.thread_lock.acquire()
            try:
                self.topic_data_buffer.append(topic_msg)
            finally:
                self.thread_lock.release()
            
    def add_engine(self, engine: BaseRuleEngine):
        for topic_name in engine.trigger_topics:
            if topic_name not in self.trigger_topic_engine_dict:
                self.trigger_topic_engine_dict[topic_name] = []
            self.trigger_topic_engine_dict[topic_name].append(engine)
            self.subscribe_topic(topic_name)
        self.subscribe_topics(engine.topics)
        
    
    # 开始把数据存进engine
    def start_record_to_engine(self, engine: BaseRuleEngine, start_time):
        logger.info(f'start record data from data_manager to engine: {engine.name}')
        for topic_name in engine.topics:
            if topic_name not in self.raw_topic_engine_dict:
                self.raw_topic_engine_dict[topic_name] = []
            self.raw_topic_engine_dict[topic_name].append(engine)

        if self.use_buffer:
            self.thread_lock.acquire()
            try:
                self.clean_over_time_msg()
                # 遍历 self.topic_data_queue
                start_copy = False
                for topic_msg in self.topic_data_buffer:
                    if not start_copy:
                        if topic_msg.receive_time >= start_time:
                            start_copy = True
                        else:
                            continue
                    
                    if topic_msg.topic in engine.topics:
                        engine.raw_topic_queue.put(topic_msg)
            finally:
                self.thread_lock.release()
    
    def end_record_to_engine(self, engine):
        for topic_name in engine.topics:
            engines = self.raw_topic_engine_dict[topic_name]
            engines.remove(engine)
        logger.info(f"Remove engine {engine} from raw_topic_engine_dict")
        
    def clean_over_time_msg(self):
        current_time = time.time()
        while self.topic_data_buffer and current_time - self.topic_data_buffer[0].receive_time > self.time_threshold:
            del self.topic_data_buffer[0]
