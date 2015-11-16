import rostopic

def get_topic_info(topic):
    topic_type, real_topic, msg_eval = rostopic.get_topic_type(topic)
    msg_class, _, _ = rostopic.get_topic_class(topic)
    return topic_type, msg_class, real_topic, msg_eval
