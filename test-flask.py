from flask import Flask
from flask import render_template

app = Flask(__name__)
app.debug = True

@app.route('/hello')
def hello_world():
    return 'Hello World!'

@app.route('/svg')
def svg():
    return render_template('index.html', name='caro', path_svg='sample.svg')

if __name__ == '__main__':
    app.run()
