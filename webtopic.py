import logging
import os
import random
import requests
import threading
import time
import signal
import sys

from flask import Flask
from flask import render_template, request, send_from_directory
from flask import jsonify, make_response, session

from rqt_topic import topic_info

import generate_dotcode
import rospy
import rostopic_funcs

app = Flask(__name__)
app.secret_key = "9876t5rdkjh34ucdsyettursg(*&";

NODE_NAME = 'webtopic'
# rospy.init_node(NODE_NAME, anonymous=True, disable_signals=True)

SVG_GENERATOR = generate_dotcode.Generator()

# TODO delete as soon as a mutex is implemented at InterProcessTopicsInfo
SUBSCRIBED_TOPICS_LOCK = threading.Lock()
TOPICS_INFO_DICT = {}


# TODO make_response is not being used properly

def generate_session_identifier():
    return str(str(time.time()) + str(random.random()))


@app.route('/favicon.ico')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'),
                               'favicon.ico',
                               mimetype='image/vnd.microsoft.icon')


def get_session_identifier():
    if 'user_id' not in session:
        session['user_id'] = generate_session_identifier()
    return session['user_id']


def get_topics_info(user_id):
    global TOPICS_INFO_DICT
    if user_id not in TOPICS_INFO_DICT:
        global NODE_NAME
        topic_info = rostopic_funcs.InterProcessTopicsInfo(NODE_NAME)
        topic_info.daemon = True
        topic_info.start()
        TOPICS_INFO_DICT[user_id] = topic_info
    return TOPICS_INFO_DICT[user_id]

@app.route('/echo')
def echo():
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
            {"error": "You must specify a topic and a msg type."}),
            400)
    return render_template('echo.html', topic=topic)

@app.route('/mobile')
@app.route('/')
def get_mobile():
    global SVG_GENERATOR, NODE_NAME
    try:
        svg_path = SVG_GENERATOR.get_current_svg(
                'static/graphs', nodes_exclude=['/' + NODE_NAME])

    except generate_dotcode.UnreachableRos:
        return make_response(
            jsonify({"error": "Unable to reach ROS, is it running?"}), 401)

    return render_template('index.html', svg_filename=svg_path)


# API

@app.route('/get_msg_type')
def get_msg_type():
    try:
        topic = request.args.getlist('topic[]')
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
            {"error": "You must specify a topic[] to get the msg type."}),
            400)
    try:
        for t in topic:
            topic_type, msg_class, real_topic, msg_eval = (
                rostopic_funcs.get_topic_info(t))
            if real_topic is not None:
                break
    except Exception as e:
        logging.exception(e)
        return make_response(jsonify({"error": "ERROR"}), 400)
    msg_struct = rostopic_funcs.get_msg_struct(topic_type)
    return jsonify({'topic_type': topic_type,
                    'real_topic': real_topic,
                    'msg_eval': msg_eval,
                    'msg_struct': msg_struct,
                    'query_topic': real_topic})


@app.route('/subscribe')
def subscribe():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.subscribe,
                                    lambda x: make_response(jsonify({})))


@app.route('/unsubscribe')
def unsubscribe():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.unsubscribe,
                                    lambda x: make_response(jsonify({})))


@app.route('/get_hz')
def get_hz():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.get_hz,
                                    lambda x: make_response(jsonify({'hz': x[0]})))


@app.route('/get_bw')
def get_bw():
    topics_info = get_topics_info(get_session_identifier())
    return _topic_template_executer(topics_info.get_bw,
                                    lambda x: make_response(jsonify({'avg': x[0]})))

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

@app.route('/get_svg_topics_and_nodes')
def get_svg_topics_and_nodes():
    global SVG_GENERATOR
    try:
        svg_name = request.args['svg_name']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'You must specify svg_name'}))
    try:
        topics, nodes = SVG_GENERATOR.get_svg_nodes_topics_lists(svg_name)
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
                {'error': 'Svg not available'}))
    return make_response(jsonify(
            {'topics': topics, 'nodes': nodes} ))

#Utils
def _topic_template_executer(func, parser):
    topics_info = get_topics_info(get_session_identifier())
    try:
        topic = request.args['topic']
    except KeyError as e:
        logging.exception(e)
        return make_response(jsonify(
            {'error': 'You must specify a topic to subscribe to.'}), 400)
    with SUBSCRIBED_TOPICS_LOCK:
        try:
            return parser(func(topic))
        except KeyError as e:
            logging.exception(e)
            return make_response(jsonify(
                {"error": "Topic not found, are you subscribed to this topic?."}),
                400)
        except ValueError as e:
            logging.exception(e)
            return make_response(jsonify({'error': 'Topic not found'}), 400)
        except Exception as e:
            logging.exception(e)
            return make_response(jsonify({}), 500)

def clean_exit():
    global TOPICS_INFO_DICT
    for t in TOPICS_INFO_DICT.values():
        t.tear_down_process()
    print("Clean exit")


if __name__ == '__main__':
    app.debug = True
    #signal.signal(signal.SIGINT, _shutdown)
    try:
        ret = app.run("0.0.0.0", threaded=55, use_reloader=False)
    finally:
        clean_exit()
