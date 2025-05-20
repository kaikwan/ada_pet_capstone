from flask import Flask
from flask_cors import CORS
import subprocess

app = Flask(__name__)
CORS(app)

@app.route('/playmeow', methods=['GET'])
def play_meow():
    subprocess.run(["python3", "/home/hello-robot/ada_pet_capstone/src/ada_pet_capstone/web_teleop/launch/play_audio.py"])
    return "Meow sound played!", 200

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8080)  
