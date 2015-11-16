import generate_dotcode
import os.path

from flask import Flask
from flask import render_template, request, send_from_directory
from flask import jsonify, make_response
import logging
import time

app = Flask(__name__)
app.debug = True

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
		file_name = "%s.svg" % time.time()
		svg_path = SVG_GENERATOR.get_current_svg('static/graphs', file_name=file_name)
    except generate_dotcode.UnreachableRos:
        return make_response(
                jsonify({"error" : "Unable to reach ROS, is it running?"}), 401)
	
    return render_template('svg-interaction.html', svg_filename=svg_path)

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
    return jsonify({'msg_type': 'odom/vel', 'topic':topic})

if __name__ == '__main__':
    app.run()
