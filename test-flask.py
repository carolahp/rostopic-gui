import os.path
import logging
import threading
import logging
import random
import time

from flask import Flask
from flask import render_template, request, send_from_directory
from flask import jsonify, make_response, session

from rqt_topic import topic_info

import generate_dotcode
import rospy
import rostopic_funcs


app = Flask(__name__)
app.secret_key="9876t5rdkjh34ucdsyettursg(*&";

NODE_NAME = 'webtopic'
#rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

SVG_GENERATOR = generate_dotcode.Generator()

#TODO delete as soon as a mutex is implemented at InterProcessTopicsInfo
SUBSCRIBED_TOPICS_LOCK = threading.Lock()
TOPICS_INFO_DICT = {}

#TODO make_response is not being used properly

def generate_session_identifier():
    return str(str(time.time()) + str(random.random()))

def get_session_identifier():
    if 'user_id' not in session:
	session['user_id'] = generate_session_identifier()
    return session['user_id']

def get_topics_info(user_id):
    global TOPICS_INFO_DICT
    if user_id not in TOPICS_INFO_DICT:
        topic_info=rostopic_funcs.InterProcessTopicsInfo()
        topic_info.start()
        TOPICS_INFO_DICT[user_id]=topic_info
    return TOPICS_INFO_DICT[user_id]
    

@app.route('/scroll')
def scroll():
    return render_template('scroll_test.html')

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
    global SVG_GENERATOR
    print("session: ", get_session())
    try:
	svg_path = SVG_GENERATOR.get_current_svg('static/graphs')
    except generate_dotcode.UnreachableRos:
        return make_response(
                jsonify({"error" : "Unable to reach ROS, is it running?"}), 401)

    return render_template('new_index.html', svg_filename=svg_path)

#API

def _topic_template_executer(func):
    topics_info = get_topics_info(get_session_identifier())
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'You must specify a topic to subscribe to.'}), 400)
    with SUBSCRIBED_TOPICS_LOCK:
        try:
            func(topic)
        except ValueError as e:
            logging.exception(e)
            return make_response(jsonify({'error': 'Topic not found'}), 400)
        except Exception as e:
            logging.exception(e)
            return make_response(jsonify({}), 500)
        return make_response(jsonify({}))


@app.route('/get_msg_type')
def get_msg_type():
    try:
        topic = request.args['topic']
    except keyerror as e:
        logging.exception(e)
        return make_response(jsonify(
                {"error": "You must specify a topic to get the msg type."}),
                400)
    try:
        topic_type, msg_class, real_topic, msg_eval = (
                rostopic_funcs.get_topic_info(topic))
    except exception as e:
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
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.subscribe)

@app.route('/unsubscribe')
def unsubscribe():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.unsubscribe)

@app.route('/get_hz')
def get_hz():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.get_hz)

@app.route('/get_bw')
def get_bw():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.get_bw)

@app.route('/get_last_msg')
def get_last_msg():
    topics_info = get_topics_info(get_session_identifier())
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': ('You must specify a topic in '
                           'order to get its last message.')}), 400)
    try:
        return make_response(jsonify(rostopic_funcs.parse_msg_as_dict(
                topics_info.get_last_msg(topic))))
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
    app.run("0.0.0.0", threaded=5)
