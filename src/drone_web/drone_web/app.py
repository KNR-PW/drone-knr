from flask import Flask, render_template
import datetime
import cv2
import os
from PIL import Image
import os

app = Flask(__name__)
# cam = cv2.VideoCapture(0)



@app.route('/')
@app.route('/index.html')
def index():
    now = datetime.datetime.now()
    timeString = now.strftime("%Y-%m-%d %H:%M:%S")
    return render_template('index.html')

@app.route('/cakes')
def cakes():
    return 'Yummy cakes!'

@app.route('/detector.html')
def detector():
    if os.path.exists("static/frame.jpg"):
        os.remove("static/frame.jpg")
    cam = cv2.VideoCapture(0)
    while not cam.isOpened():
        i=0
    ret, frame = cam.read()
    frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite("static/frame.jpg", frame)
    return render_template('detector.html')

# @app.route('/detector.html/load_img')
# def load_img():

@app.route('/hello/<name>')
def hello(name):
    if os.path.exists("static/frame.jpg"):
        os.remove("static/frame.jpg")
    cam = cv2.VideoCapture(0)
    while not cam.isOpened():
        i=0
    ret, frame = cam.read()
    cv2.imwrite("static/frame.jpg", frame)
    return render_template('page.html', name=name)


def main():
    print(os.path.dirname(__file__))

    app.run(debug=True, host='0.0.0.0')

if __name__ == '__main__':
    main()