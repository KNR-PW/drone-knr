from flask import Flask, g, request, flash
import uuid
import os
import sqlite3
import json
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# db shit: https://flask.palletsprojects.com/en/2.3.x/patterns/sqlite3/

DATABASE = 'ayy.db'

def get_db():
    db = getattr(g, '_database', None)
    if db is None:
        db = g._database = sqlite3.connect(DATABASE)
    db.row_factory = sqlite3.Row
    return db

@app.teardown_appcontext
def close_connection(exception):
    db = getattr(g, '_database', None)
    if db is not None:
        db.close()

def init_db():
    with app.app_context():
        db = get_db()
        with app.open_resource('schema.sql', mode='r') as f:
            db.cursor().executescript(f.read())
        db.commit()


UPLOAD_FOLDER = 'testupload'
ALLOWED_EXTENSIONS = { 'png', 'jpg', 'jpeg' }


def extension(filename):
    return filename.split('.')[-1]

def upload_data():
    file = request.files['photo']
    ext = extension(file.filename)
    if ext not in ALLOWED_EXTENSIONS:
        return ('fuck you, faggot', 400)

    filename = f'{str(uuid.uuid4())}.{ext}'
    save_path = os.path.join(UPLOAD_FOLDER, filename)
    file.save(save_path)
    
    f = request.form
    db = get_db()
    cur = db.cursor()
    cur.execute(
        'INSERT INTO detection(date, lat, lon, image_name) VALUES (?, ?, ?, ?)',
        (
            f.get('date', type=str),
            f.get('lat', type=float),
            f.get('lon', type=float),
            filename
        )
    )

    detection_id = cur.lastrowid
    for tree_str in f.getlist('trees', type=str):
        # parse it
        tree = json.loads(tree_str) 

        cur.execute(
            'INSERT INTO tree(detection, color) VALUES (?, ?)',
            (
                detection_id,
                tree['color']
            )
        )

    db.commit()
    return ('created', 201)
    
def data():
    db = get_db()
    cur = db.cursor()
    rs = cur.execute('SELECT d.id AS detection_id, d.date, d.lat, d.lon, d.image_name, t.id AS tree_id, t.color FROM detection d LEFT JOIN tree t ON t.detection = d.id GROUP BY detection_id, tree_id').fetchall()
    
    # i don't know the idiomatic way to do this...
    # collect detections + trees
    # unfortunately, we can't sql ORDER it. either order it here or clientside
    dets = {}
    for r in rs:
        did = r['detection_id']
        if did not in dets:
            dets[did] = {
                'detection_id': did,
                'date': r['date'],
                'lat': r['lat'],
                'lon': r['lon'],
                'photo': r['image_name'],
                'trees': []
            }

        if r['tree_id'] != None:
            det = dets[did]
            det['trees'].append({ 'tree_id': r['tree_id'], 'color': r['color'] })        
  
    return list(dets.values())



@app.route('/', methods=['GET', 'POST'])
def get_data():
    if request.method == 'POST':
        return upload_data()
    elif request.method == 'GET':
        return data()