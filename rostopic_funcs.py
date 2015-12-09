import multiprocessing

import roslib
import rosmsg
import rospy
import rostopic

from rqt_topic.topic_info import TopicInfo

class TopicsInfo():
    def __init__(self):
        self.topics = {}

    def subscribe(self, topic):
        if topic in self.topics:
            if not self.topics[topic].monitoring:
                self.topics[topic].start_monitoring()
            return
        topic_type, _, topic_name, _ = get_topic_info(topic)
        if not topic_type or not topic_name:
            raise ValueError("Topic not found")
        topic_obj = TopicInfo(topic_name, topic_type)

        topic_obj.start_monitoring()

        self.topics[topic_name] = topic_obj
        if topic_name != topic:
            self.topics[topic] = topic_obj

    def unsubscribe(self, topic):
	self.topics[topic].stop_monitoring()

    def get_last_msg(self, topic):
        return self.topics[topic].last_message

    def get_bw(self, topic):
        print(self.topics[topic].get_bw())
        return self.topics[topic].get_bw()

    def get_hz(self, topic):
        return self.topics[topic].get_hz()

class InterProcessPublisher(multiprocessing.Process):
    pass


class InterProcessTopicsInfo(multiprocessing.Process):
    _GET_LAST_MSG = 'get_last_msg'
    _SUBSCRIBE = 'subscribe'
    _GET_BW = 'get_bw'
    _GET_HZ = 'get_hz'
    _UNSUBSCRIBE = 'unsubscribe'
    _TEAR_DOWN = 'tear_down'
    def __init__(self, node_name="interprocesstopicsinfo"):
        self.to_process_queue = multiprocessing.Queue()
        self.from_process_queue = multiprocessing.Queue()
        self.node_name = node_name
        super(InterProcessTopicsInfo, self).__init__()

    def _start_node(self):
        rospy.init_node(self.node_name, anonymous=True, disable_signals=True)

    def _raise_on_exception(self, ret):
        if isinstance(ret, Exception):
            raise ret
        return ret

    def run(self):
        self._start_node()
        self.topics_info = TopicsInfo()
        while True:
            req, param = self.to_process_queue.get()
            try:
                if req == self._GET_LAST_MSG:
                    ret = self.topics_info.get_last_msg(param)
                    self.from_process_queue.put(ret)

                elif req == self._SUBSCRIBE:
                    ret = self.topics_info.subscribe(param)
                    self.from_process_queue.put(ret)

                elif req == self._GET_BW:
                    ret = self.topics_info.get_bw(param)
                    self.from_process_queue.put(ret)

                elif req == self._GET_HZ:
                    ret = self.topics_info.get_hz(param)
                    self.from_process_queue.put(ret)

		elif req == self._UNSUBSCRIBE:
		    ret = self.topics_info.unsubscribe(param)
                    self.from_process_queue.put(ret)

                elif req == self._TEAR_DOWN:
                    print("Tearing down")
                    break

                else:
                    self.from_process_queue.put("Not a valid operation (%s)" % req)
            except Exception as e:
                self.from_process_queue.put(e)

        print("Shutting down")
        rospy.signal_shutdown("Shutting down")

    #TODO mutex in all the methods
    def _send(self, s):
        self.to_process_queue.put(s)
        return self.from_process_queue.get()

    def get_last_msg(self, topic):
        self.to_process_queue.put((self._GET_LAST_MSG, topic))
        return self._raise_on_exception(self.from_process_queue.get())

    def subscribe(self, topic):
        self.to_process_queue.put((self._SUBSCRIBE, topic))
        return self._raise_on_exception(self.from_process_queue.get())

    def get_bw(self, topic):
        self.to_process_queue.put((self._GET_BW, topic))
        return self._raise_on_exception(self.from_process_queue.get())

    def get_hz(self, topic):
        self.to_process_queue.put((self._GET_HZ, topic))
        return self._raise_on_exception(self.from_process_queue.get())

    def unsubscribe(self, topic):
        self.to_process_queue.put((self._UNSUBSCRIBE, topic))
        return self._raise_on_exception(self.from_process_queue.get())

    def tear_down_process(self):
        self.to_process_queue.put((self._TEAR_DOWN, None))

def get_topic_info(topic):
    topic_type, real_topic, msg_eval = rostopic.get_topic_type(topic)
    msg_class, _, _ = rostopic.get_topic_class(topic)
    return topic_type, msg_class, real_topic, msg_eval

#Got from topic_widget
def _extract_array_info(type_str):
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)
        else:
            array_size = 0

    return type_str, array_size

def parse_msg_as_dict(msg, name='msg'):
    ret_list = []
    ret = {}
    if msg is None:
        return {}
    if hasattr(msg, '__slots__') and hasattr(msg, '_slot_types'):
        for slot_name in msg.__slots__:
            ret[slot_name] = parse_msg_as_dict(getattr(msg, slot_name),
                                               slot_name)
        return ret
    elif type(msg) in (list, tuple) and (len(msg) > 0):
        for el in msg:
            #TODO check if I need to use _extract_array_info
            ret_list.append(parse_msg_as_dict(el, name))
        return ret_list
    else:
        return repr(msg)


def get_msg_struct(msg_name):
    try:
        msg_class = roslib.message.get_message_class(msg_name)
    except (ValueError, TypeError):
        return
    return get_msg_struct_(msg_class, {})

def get_msg_struct_(msg_class, ret):
    if hasattr(msg_class, '__slots__') and hasattr(msg_class, '_slot_types'):
        for slot_name, slot_type in zip(msg_class.__slots__,
                                        msg_class._slot_types):
            try:
                slot_class = roslib.message.get_message_class(slot_type)
            except (ValueError, TypeError):
                slot_class = None
            if slot_class:
                ret[slot_name] = get_msg_struct_(slot_class, {})
            else:
                ret[slot_name] = slot_type
    else:
        #TODO test with arrays
        print("ASDWQASD")
        base_type_str, arr_size = _extract_array_info(type_name)
    return ret

    #msg_text = rosmsg.get_msg_text(msg_class)
