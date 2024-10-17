from multiprocessing import Queue
from multiprocessing.connection import wait
import genpy
from rospy.msg import AnyMsg

import time

from loguru import logger
from rosbag import Bag


class BaseRuleEngine:
    trigger_topics = []
    
    def __init__(self, topics, data_manager):
        self.trigger_topic_queue = Queue()
        self.raw_topic_queue = Queue()
        self.trigger_start_time = None
        self.running = True
        self.topics = topics
        self.data_manager = data_manager
        # self.use_buffer = use_buffer
        self.buffer_time = 10 # 秒
        
        
    def file_name_(self):
        pass
    
    # 触发条件
    def condition_start(self, trigger_msg ):
        pass
    
    def condition_end(self, trigger_msg ):
        pass

    def trigger_start(self,):
        self.trigger_start_time = time.time()
        self.bag = Bag(self.file_name_(), 'w')
        self.data_manager.start_record_to_engine(self, time.time()-self.buffer_time)
        
        logger.info(f"engine { self.name } start trigger ")
    
    def trigger(self,  topic_message):
        if self.bag is not None and not self.bag._file:
            logger.warning(f"bag file is not open")
            return
            
        
        receive_time = genpy.Time.from_sec(topic_message.receive_time)
        if topic_message.type == AnyMsg:
            self.bag.write(topic_message.topic, topic_message.msg, t=receive_time, raw=True)
        else:
            self.bag.write(topic_message.topic, topic_message.msg, t=receive_time)
            # logger.debug(f"write msg {topic_message.topic} {topic_message.msg} to bag ")
    
    def trigger_end(self):
        logger.info(f'engine {self.name} trigger end')
        if self.trigger_start_time is not None:
            self.data_manager.end_record_to_engine(self)

            # self.bag.reindex()
            # self.bag.flush()
            self.bag.close()

            
            logger.info(f"{self.bag.filename} saved")
            self.trigger_start_time = None
        # 计算下一次 触发时间 或者 等待下次触发条件

    def stop(self):
        self.running = False

    def run(self):
        # 接受 topic_message 消息，开始处理
        while True:
            if not self.running:
                self.trigger_end()
                break
            try:
                
                ready = wait([self.trigger_topic_queue._reader, self.raw_topic_queue._reader], 1)
                trigger_msg = None
                if ready:
                    for q in ready:
                        if q == self.trigger_topic_queue._reader:
                            trigger_msg = self.trigger_topic_queue.get()

                        elif q == self.raw_topic_queue._reader:
                            raw_msg = self.raw_topic_queue.get()
                            self.trigger(raw_msg)

                        else:
                            logger.error(f'{self.name} wait error')

                if self.trigger_start_time is None:
                    # 未开始录制
                    if self.condition_start(trigger_msg):
                        # 如果满足开始条件
                        self.trigger_start()
                else:
                    if self.condition_end(trigger_msg):
                        # 如果满足结束条件
                        self.trigger_end()

            except  Exception as e:
                logger.warning(f"{self.name} error: {e}")
                continue
            
