from flask import Flask, render_template, request, redirect, url_for, send_file
import sqlite3
import base64
import os
import cv2

app = Flask(__name__)

# Database setup
DB_NAME = 'faces.db'

def create_database():
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS images (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            filename TEXT,
            image_data BLOB
        )
    ''')
    conn.commit()
    conn.close()

create_database() 

# Function for face detection
def encontra_face(filename):
    cascade_path = os.path.join(os.path.dirname(__file__), 'haarcascade_frontalface_default.xml')
    face_cascade = cv2.CascadeClassifier(cascade_path)
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        print("Face encontrada!")

    output_path = os.path.join(app.root_path, "temp", "faces_temp.jpg")
    cv2.imwrite(output_path, img)
    
    return output_path

# Route for the home page
@app.route('/')
def index():
    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM images')
    images = cursor.fetchall()
    conn.close()
    
    return render_template('index.html', images=images)

# Route for uploading an image and detecting faces
@app.route('/upload', methods=['POST'])
def upload():
    if 'image' in request.files:
        image = request.files['image']
        filename = image.filename
        filepath = os.path.join(app.root_path, 'temp', filename)
        image.save(filepath)
        print(f"File saved to: {filepath}")

        found_face = encontra_face(filepath)
        print("Face detected!")

        # Read the faces_temp.jpg file and encode it in base64
        with open(filepath, 'rb') as file:
            image_data = file.read()
            encoded_image = base64.b64encode(image_data).decode('utf-8')

        # Save the image and filename to the database
        conn = sqlite3.connect(DB_NAME)
        cursor = conn.cursor()
        cursor.execute('INSERT INTO images (filename, image_data) VALUES (?, ?)', (filename, encoded_image))
        conn.commit()
        conn.close()

        # Delete the faces_temp.jpg file
        os.remove(filepath)
        print("File deleted.")

    return redirect(url_for('index'))

# Route for displaying the image with detected faces
@app.route('/temp_image')
def temp_image():
    temp_image_path = os.path.join(app.root_path, "temp", "faces_temp.jpg")
    return send_file(temp_image_path, mimetype='image/jpeg')

if __name__ == '__main__':
    app.run(debug=True)