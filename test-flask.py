import generate_dotcode
import os.path

from flask import Flask
from flask import render_template, send_from_directory
import logging

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
        return "Unable to reach ROS, is it running?"
    return render_template('index.html', name='caro',
                           svg_filename=os.path.basename(svg_path))

#API
@app.route('/get_msg_type')
def get_msg_type():
    return 'odom/vel'

if __name__ == '__main__':
    app.run()
