import os.path
import logging
import threading
import logging
import time

from flask import Flask
from flask import render_template, request, send_from_directory
from flask import jsonify, make_response

from rqt_topic import topic_info

import generate_dotcode
import rospy
import rostopic_funcs


app = Flask(__name__)

NODE_NAME = 'webtopic'
#rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

SVG_GENERATOR = generate_dotcode.Generator()
SUBSCRIBED_TOPICS = {}
SUBSCRIBED_TOPICS_LOCK = threading.Lock()

#TODO make_response is not being used properly

@app.route('/hello')
def hello_world():
    return 'Hello World!'

@app.route('/svg')
def svg():
    global SVG_GENERATOR
    try:
        svg_path = SVG_GENERATOR.get_current_svg('static/graphs')
    except generate_dotcode.UnreachableRos:
        return make_response(
                jsonify({"error" : "Unable to reach ROS, is it running?"}), 401)
    return render_template('index.html', name='caro',
                           svg_filename=os.path.basename(svg_path))


@app.route('/interaction')
def interaction():
    global SVG_GENERATOR
    try:
	svg_path = SVG_GENERATOR.get_current_svg('static/graphs')
    except generate_dotcode.UnreachableRos:
        return make_response(
                jsonify({"error" : "Unable to reach ROS, is it running?"}), 401)

    return render_template('svg-interaction.html', svg_filename=svg_path)

@app.route('/mobile')
def get_mobile():
    return render_template('new_index.html')

#API
@app.route('/get_msg_type')
def get_msg_type():
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {"error": "You must specify a topic to get the msg type."}),
                400)
    try:
        topic_type, msg_class, real_topic, msg_eval = (
                rostopic_funcs.get_topic_info(topic))
    except Exception as e:
        logging.exception(e)
        return make_response(jsonify({"error": "ERROR"}), 400)
    msg_struct = rostopic_funcs.get_msg_struct(topic_type)
    return jsonify({'topic_type': topic_type,
                    'real_topic': real_topic,
                    'msg_eval': msg_eval,
                    'msg_struct': msg_struct,
                    'query_topic':topic})

@app.route('/subscribe')
def subscribe():
    global SUBSCRIBED_TOPICS, SUBSCRIBED_TOPICS_LOCK
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'You must specify a topic to subscribe to.'}), 400)
    with SUBSCRIBED_TOPICS_LOCK:
        if topic in SUBSCRIBED_TOPICS:
            topic_obj = SUBSCRIBED_TOPICS[topic]
            if not topic_obj.monitoring:
                topic_obj.start_monitoring()
            return make_response(jsonify({}))
        try:
            topic_type, _, topic_name, _ = rostopic_funcs.get_topic_info(topic)
            if not topic_type or not topic_name:
                return make_response(jsonify(
                        {'error': 'Unable to subscribe to %s' % topic}), 400)
            topic_obj = topic_info.TopicInfo(topic_name, topic_type)
            #import pdb; pdb.set_trace()
            topic_obj.start_monitoring()
            SUBSCRIBED_TOPICS[topic] = topic_obj

        except Exception as e:
            logging.exception(e)
            return make_response(jsonify({}), 500)
        return make_response(jsonify({}))

@app.route('/get_last_msg')
def get_last_msg():
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': ('You must specify a topic in '
                           'order to get its last message.')}), 400)
    try:
        return make_response(jsonify(rostopic_funcs.parse_msg_as_dict(
                SUBSCRIBED_TOPICS[topic].last_message)))
    except AttributeError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'No message received yet.'}, 400))
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'Not subscribed to this topic'}))
    except Exception as e:
        logging.exception(e)
        return make_response(jsonify({}), 500)


if __name__ == '__main__':
    app.debug = True
    try:
        rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True,
                        disable_rosout=True, disable_rostime=True)
        app.run("0.0.0.0", threaded=5)
    finally:
        rospy.signal_shutdown("shutting down")
