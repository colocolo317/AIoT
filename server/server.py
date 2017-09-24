from flask import Flask
from flask import jsonify
from flask import request


app = Flask(__name__)


@app.route('/')
def index():
    print(request.get_json)
    return "Hello, World!"


@app.route('/send')
def send():
    ret = {"action": "send"}
    data = request.get_json()
    print(data)
    if data is None:
        return jsonify(ret), 400
    return jsonify(ret), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=True)
