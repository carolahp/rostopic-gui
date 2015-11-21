import roslib
import rosmsg
import rostopic

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

def get_msg_struct(msg_name, msg=None):
    try:
        msg_class = roslib.message.get_message_class(msg_name)
    except (ValueError, TypeError):
        return
    return get_msg_struct_(msg_class, {})

def get_msg_struct_(msg_class, ret):
    slots = msg_class.__slots__
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
        print("ASDWQASD")
        base_type_str, arr_size = _extract_array_info(type_name)
    return ret

    #msg_text = rosmsg.get_msg_text(msg_class)
