import os.path
import logging

from flask import Flask
from flask import render_template, request, send_from_directory
from flask import jsonify, make_response

import generate_dotcode
import rostopic_funcs

import logging
import time

app = Flask(__name__)

SVG_GENERATOR = generate_dotcode.Generator()

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
    return jsonify({'topic_type': topic_type,
                    'real_topic': real_topic,
                    'msg_eval': msg_eval,
                    'query_topic':topic})

if __name__ == '__main__':
    app.debug = True
    app.run("0.0.0.0")
