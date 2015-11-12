from generate_dotcode import Generator
import os.path

from flask import Flask
from flask import render_template, send_from_directory
import time

app = Flask(__name__)
app.debug = True

LAST_SVG_TS = 0
TIME_TO_REFRESH_SVG = 5 #Segs

def _write_svg(path):
    g = Generator()
    g.generate(path)

def _get_current_svg():
    #TODO mutex
    global LAST_SVG_TS
    global TIME_TO_REFRESH_SVG

    now = time.time()
    svg_path = "static/graphs/%s.svg" % TIME_TO_REFRESH_SVG
    if now - LAST_SVG_TS > TIME_TO_REFRESH_SVG:
        svg_path = "static/graphs/%s.svg" % now
        _write_svg(svg_path)
    return os.path.basename(svg_path)

@app.route('/hello')
def hello_world():
    return 'Hello World!'

@app.route('/svg')
def svg():
    svg_path = _get_current_svg()
    print(svg_path)
    return render_template('index.html', name='caro', svg_filename=svg_path, svg="qwe", path="asd")

if __name__ == '__main__':
    app.run()
